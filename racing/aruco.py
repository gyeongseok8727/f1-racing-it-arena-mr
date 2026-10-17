"""Course ArUco markers -> absolute vehicle pose (PLAN.md §3, track/README §3.1).

Markers: DICT_4X4_50 IDs 0/20/30/45, board 10 cm, *code* 7 cm (corners returned by the
detector are the code's), centre 0.40 m above the floor, facing approaching cars.
Detector uses minMarkerDistanceRate = 0.02 — the 0.05 default drops valid markers.

Frames: base = rear axle, x forward, y left, z up. OpenCV camera: x right, y down, z forward.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CODE_SIZE = 0.07
COURSE_IDS = (0, 20, 30, 45)


@dataclass
class CameraModel:
    K: np.ndarray  # 3x3
    dist: np.ndarray  # (5,)
    # camera optical centre in the base frame and its upward pitch
    x: float = 0.12
    y: float = 0.0
    z: float = 0.15
    pitch_up: float = math.radians(10)

    @classmethod
    def from_fov(cls, width: int, height: int, hfov_deg: float, **kw) -> "CameraModel":
        f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
        K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], dtype=float)
        return cls(K, np.zeros(5), **kw)

    def T_base_cam(self) -> np.ndarray:
        # camera axes expressed in base: z_cam = forward, x_cam = -left, y_cam = -up
        R0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        c, s = math.cos(-self.pitch_up), math.sin(-self.pitch_up)
        Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        T = np.eye(4)
        T[:3, :3] = Ry @ R0
        T[:3, 3] = (self.x, self.y, self.z)
        return T


def marker_world_T(pose: dict, center_z: float = 0.40) -> np.ndarray:
    """scene.json marker pose -> 4x4 world_T_marker in OpenCV marker convention
    (x right as seen by a viewer, y up, z = outward normal toward the viewer)."""
    yaw = pose["yaw_rad"]
    z = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y = np.array([0.0, 0.0, 1.0])
    x = np.cross(y, z)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = (pose["x"], pose["y"], center_z)
    return T


def marker_object_points(size: float = CODE_SIZE) -> np.ndarray:
    h = size / 2
    # detector corner order: top-left, top-right, bottom-right, bottom-left
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def make_detector():
    import cv2

    params = cv2.aruco.DetectorParameters()
    params.minMarkerDistanceRate = 0.02
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.ArucoDetector(d, params)


@dataclass
class MarkerFix:
    marker_id: int
    x: float
    y: float
    yaw: float
    distance: float  # camera -> marker, m
    reproj_err: float  # px


class ArucoLocalizer:
    def __init__(self, scene: dict, cam: CameraModel, max_distance: float = 1.6, max_reproj: float = 2.0):
        aru = scene["aruco_markers"]
        cz = aru.get("mount_center_height_m", 0.40)
        self.world = {m["id"]: marker_world_T(m["pose"], cz) for m in aru["markers"]}
        self.cam = cam
        self.detector = make_detector()
        self.max_distance = max_distance
        self.max_reproj = max_reproj
        self._T_cam_base = np.linalg.inv(cam.T_base_cam())

    def detect(self, gray: np.ndarray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        # dedupe (0.02 rate can return the same ID twice): keep the largest
        best = {}
        for c, i in zip(corners, ids.flatten()):
            i = int(i)
            area = abs(cv2_contour_area(c.reshape(4, 2)))
            if i in self.world and (i not in best or area > best[i][1]):
                best[i] = (c.reshape(4, 2), area)
        return [(i, c) for i, (c, _) in best.items()]

    def fixes(self, gray: np.ndarray, yaw_hint: float | None = None) -> list[MarkerFix]:
        """Vehicle pose from each visible course marker.

        A 7 cm marker at 1-2 m is only ~25-40 px wide; its PnP *rotation* is off by a few
        degrees, which moves the derived position by distance x angle (6-13 cm). The PnP
        *translation* (bearing + range) is much better. With `yaw_hint` (PF/odometry yaw, good
        to ~1 deg from lidar) only the translation is used: position error ~ 2-3 cm.
        """
        import cv2

        out = []
        obj = marker_object_points()
        for mid, img_pts in self.detect(gray):
            ok, rvec, tvec = cv2.solvePnP(obj, img_pts.astype(np.float64), self.cam.K, self.cam.dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(obj, rvec, tvec, self.cam.K, self.cam.dist)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img_pts) ** 2, axis=1))))
            dist = float(np.linalg.norm(tvec))
            if err > self.max_reproj or dist > self.max_distance:
                continue
            if yaw_hint is None:
                T_cam_marker = np.eye(4)
                T_cam_marker[:3, :3] = cv2.Rodrigues(rvec)[0]
                T_cam_marker[:3, 3] = tvec.ravel()
                T_world_base = self.world[mid] @ np.linalg.inv(T_cam_marker) @ self._T_cam_base
                x, y = T_world_base[0, 3], T_world_base[1, 3]
                yaw = math.atan2(T_world_base[1, 0], T_world_base[0, 0])
            else:
                # marker centre in base frame, rotated into the world with the hinted yaw
                Tbc = self.cam.T_base_cam()
                p_base = Tbc[:3, :3] @ tvec.ravel() + Tbc[:3, 3]
                c, s = math.cos(yaw_hint), math.sin(yaw_hint)
                mw = self.world[mid][:3, 3]
                x = mw[0] - (c * p_base[0] - s * p_base[1])
                y = mw[1] - (s * p_base[0] + c * p_base[1])
                yaw = yaw_hint
            out.append(MarkerFix(mid, float(x), float(y), float(yaw), dist, err))
        return out


def cv2_contour_area(pts: np.ndarray) -> float:
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def render_marker_view(marker_png: np.ndarray, T_world_marker: np.ndarray, T_world_base: np.ndarray,
                       cam: CameraModel, size_wh=(848, 480), board: float = 0.10) -> np.ndarray:
    """Synthetic camera image of one marker board (for tests and detector tuning)."""
    import cv2

    h = board / 2
    board_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    T_cam_marker = np.linalg.inv(T_world_base @ cam.T_base_cam()) @ T_world_marker
    rvec = cv2.Rodrigues(T_cam_marker[:3, :3])[0]
    proj, _ = cv2.projectPoints(board_pts, rvec, T_cam_marker[:3, 3], cam.K, cam.dist)
    src = np.array([[0, 0], [marker_png.shape[1], 0], [marker_png.shape[1], marker_png.shape[0]],
                    [0, marker_png.shape[0]]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, proj.reshape(4, 2).astype(np.float32))
    bg = np.full((size_wh[1], size_wh[0]), 90, np.uint8)
    warped = cv2.warpPerspective(marker_png, H, size_wh, borderValue=0)
    mask = cv2.warpPerspective(np.full(marker_png.shape[:2], 255, np.uint8), H, size_wh) > 0
    bg[mask] = warped[mask]
    return bg

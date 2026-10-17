"""Official track assets: closed paths, occupancy map, scene metadata."""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_DIR = REPO_ROOT / "track" / "official"

# map.png pixel values (see track/official/SOURCE.md)
PX_WALL = 0
PX_OUTSIDE = 205
PX_FREE = 254
PX_GRASS = 150  # only in map_with_grass.png


@dataclass
class TrackPath:
    """Polyline with arc length. Closed paths wrap around at `length`."""

    xy: np.ndarray  # (N, 2)
    s: np.ndarray  # (N,)
    width: np.ndarray  # (N,)
    closed: bool = True
    heading: np.ndarray = field(init=False)
    curvature: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.heading = self._compute_heading()
        self.curvature = self._compute_curvature()

    @property
    def length(self) -> float:
        if self.closed:
            return float(self.s[-1] + np.linalg.norm(self.xy[0] - self.xy[-1]))
        return float(self.s[-1])

    def __len__(self) -> int:
        return len(self.s)

    def _neighbors(self, k: int) -> tuple[np.ndarray, np.ndarray]:
        n = len(self.xy)
        idx = np.arange(n)
        if self.closed:
            return self.xy[(idx - k) % n], self.xy[(idx + k) % n]
        return self.xy[np.clip(idx - k, 0, n - 1)], self.xy[np.clip(idx + k, 0, n - 1)]

    def _compute_heading(self) -> np.ndarray:
        prev, nxt = self._neighbors(1)
        d = nxt - prev
        return np.arctan2(d[:, 1], d[:, 0])

    def _compute_curvature(self, k: int = 3) -> np.ndarray:
        """Signed curvature from the circle through points i-k, i, i+k (left turn > 0)."""
        a, c = self._neighbors(k)
        b = self.xy
        cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        ab = np.linalg.norm(b - a, axis=1)
        bc = np.linalg.norm(c - b, axis=1)
        ac = np.linalg.norm(c - a, axis=1)
        denom = ab * bc * ac
        kappa = np.where(denom > 1e-12, 2.0 * cross / np.maximum(denom, 1e-12), 0.0)
        return kappa

    def nearest_index(self, x: float, y: float, hint: int | None = None, window: int = 150) -> int:
        """Nearest vertex. With `hint`, search only +-window around it (fast, avoids jumping
        to a parallel section of the track)."""
        n = len(self.xy)
        if hint is None:
            d2 = np.sum((self.xy - (x, y)) ** 2, axis=1)
            return int(np.argmin(d2))
        idx = np.arange(hint - window, hint + window + 1)
        idx = idx % n if self.closed else np.clip(idx, 0, n - 1)
        d2 = np.sum((self.xy[idx] - (x, y)) ** 2, axis=1)
        return int(idx[np.argmin(d2)])

    def index_at_s(self, s: float) -> int:
        if self.closed:
            s = s % self.length
        return int(min(np.searchsorted(self.s, s), len(self.s) - 1))

    def advance(self, i: int, distance: float) -> int:
        """Index roughly `distance` metres ahead of vertex i along the path."""
        return self.index_at_s(self.s[i] + distance)

    def offset(self, d: float) -> "TrackPath":
        """Parallel path shifted d metres to the left (negative = right)."""
        nrm = np.stack([-np.sin(self.heading), np.cos(self.heading)], axis=1)
        xy = self.xy + d * nrm
        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        return TrackPath(xy, s, self.width.copy(), self.closed)


def load_path(csv_path: Path | str, closed: bool = True) -> TrackPath:
    xs, ys, ss, ws = [], [], [], []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
            ss.append(float(row["s_m"]))
            ws.append(float(row.get("track_width_m") or row.get("width_m")))
    return TrackPath(np.column_stack([xs, ys]), np.array(ss), np.array(ws), closed)


def load_centerline(official_dir: Path = OFFICIAL_DIR) -> TrackPath:
    return load_path(official_dir / "centerline.csv", closed=True)


def load_branch(i: int, official_dir: Path = OFFICIAL_DIR) -> TrackPath:
    return load_path(official_dir / f"branch_{i}.csv", closed=False)


def load_scene(official_dir: Path = OFFICIAL_DIR) -> dict:
    with open(official_dir / "scene.json", encoding="utf-8") as f:
        return json.load(f)


class OccupancyMap:
    """ROS map_server style grid. World (x, y) metres <-> image (row, col)."""

    def __init__(self, image: np.ndarray, resolution: float, origin: tuple[float, float]):
        self.image = image
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.height, self.width = image.shape
        self.occupied = (image == PX_WALL) | (image == PX_OUTSIDE)

    @classmethod
    def load(cls, yaml_path: Path | str) -> "OccupancyMap":
        import cv2
        import yaml

        yaml_path = Path(yaml_path)
        meta = yaml.safe_load(yaml_path.read_text())
        img = cv2.imread(str(yaml_path.parent / meta["image"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(yaml_path.parent / meta["image"])
        return cls(img, meta["resolution"], meta["origin"][:2])

    def world_to_px(self, x, y):
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        row = self.height - 1 - np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        return row, col

    def value_at(self, x, y):
        """Pixel value at world points; out-of-image counts as PX_OUTSIDE."""
        row, col = self.world_to_px(x, y)
        inside = (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        out = np.full(np.shape(row), PX_OUTSIDE, dtype=np.uint8)
        out[inside] = self.image[row[inside], col[inside]]
        return out

    def is_occupied(self, x, y):
        v = self.value_at(x, y)
        return (v == PX_WALL) | (v == PX_OUTSIDE)

    def raycast(self, x: float, y: float, angles: np.ndarray, max_range: float) -> np.ndarray:
        """Range to first occupied cell along each world-frame angle (simple ray marching)."""
        step = self.resolution * 0.75
        r = np.arange(step, max_range + step, step)
        px = x + np.outer(np.cos(angles), r)
        py = y + np.outer(np.sin(angles), r)
        hit = self.is_occupied(px, py)
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(r) - 1)
        return r[first]


def grid_slot_pose(slot: int, scene: dict | None = None) -> tuple[float, float, float]:
    scene = scene or load_scene()
    s = scene["starting_grid"]["slots"][slot]
    return s["x"], s["y"], s["yaw_rad"]


def wrap_angle(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi

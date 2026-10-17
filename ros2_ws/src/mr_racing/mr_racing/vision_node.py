"""Vision node: one owner of the camera. Start light until GO, then course ArUco fixes.

Pub  /start_light/go   std_msgs/Bool   (True once, re-sent at 5 Hz after GO)
     /start_light/state std_msgs/String (waiting/armed/go + lamp counts, for tuning)
     /aruco/fix        geometry_msgs/PoseStamped (map frame, rear axle)
Sub  /pf/pose          yaw hint for ArUco (see racing/aruco.py)
Srv  ~/reset_start     re-arm the start detector

Start-light ROI: [x, y, w, h] in pixels around the light as seen from the grid slot.
Set it on race day from a saved frame (`save_frame` param writes one JPEG at startup).
"""
import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from racing.aruco import ArucoLocalizer, CameraModel
from racing.start_light import ColorThresholds, StartDetector, StartState, read_lamps
from racing.track import load_scene


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        P = self.declare_parameter
        self.device = P("device", 0).value
        w, h, fps = P("width", 848).value, P("height", 480).value, P("fps", 30).value
        self.roi = list(P("start_roi", [0, 0, 848, 240]).value)
        hfov = P("hfov_deg", 69.4).value
        K = P("camera_matrix", [0.0] * 9).value
        dist = P("dist_coeffs", [0.0] * 5).value
        cam_kw = dict(x=P("cam_x", 0.12).value, y=P("cam_y", 0.0).value, z=P("cam_z", 0.15).value,
                      pitch_up=math.radians(P("cam_pitch_deg", 10.0).value))
        self.save_frame = P("save_frame", "").value
        th = ColorThresholds(v_min=P("light_v_min", 170).value, s_min=P("light_s_min", 90).value,
                             min_area_px=P("light_min_area", 12).value)
        self.th = th

        cam = (CameraModel(np.array(K, float).reshape(3, 3), np.array(dist, float), **cam_kw)
               if any(K) else CameraModel.from_fov(w, h, hfov, **cam_kw))
        self.aruco = ArucoLocalizer(load_scene(), cam)
        self.detector = StartDetector()
        self.yaw_hint = None

        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # auto exposure ON: spotlights make brightness uneven (docs/venue)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        if not self.cap.isOpened():
            self.get_logger().error(f"camera {self.device} not opened")

        self.pub_go = self.create_publisher(Bool, "start_light/go", 10)
        self.pub_light = self.create_publisher(String, "start_light/state", 10)
        self.pub_fix = self.create_publisher(PoseStamped, "aruco/fix", 10)
        self.create_subscription(PoseWithCovarianceStamped, "pf/pose", self.on_pose, 10)
        self.create_service(Trigger, "~/reset_start", self.srv_reset)
        self.create_timer(1.0 / fps, self.on_frame)
        self.create_timer(0.2, self.repeat_go)
        self._saved = False

    def on_pose(self, msg):
        q = msg.pose.pose.orientation
        self.yaw_hint = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)

    def srv_reset(self, req, res):
        self.detector.reset()
        res.success, res.message = True, "start detector re-armed"
        return res

    def repeat_go(self):
        if self.detector.state is StartState.GO:
            self.pub_go.publish(Bool(data=True))

    def on_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        if self.save_frame and not self._saved:
            cv2.imwrite(self.save_frame, frame)
            self._saved = True
            self.get_logger().info(f"saved {self.save_frame}")

        if self.detector.state is not StartState.GO:
            x, y, w, h = self.roi
            reading = read_lamps(frame[y:y + h, x:x + w], self.th)
            st = self.detector.update(reading)
            self.pub_light.publish(String(data=json.dumps(
                {"state": st.value, "reds": reading.reds, "green": reading.green})))
            if st is StartState.GO:
                self.get_logger().info("START LIGHT: GO")
                self.pub_go.publish(Bool(data=True))
            return  # skip ArUco until the race has started (CPU)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for f in self.aruco.fixes(gray, self.yaw_hint):
            m = PoseStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.pose.position.x, m.pose.position.y = f.x, f.y
            m.pose.orientation.z, m.pose.orientation.w = math.sin(f.yaw / 2), math.cos(f.yaw / 2)
            self.pub_fix.publish(m)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

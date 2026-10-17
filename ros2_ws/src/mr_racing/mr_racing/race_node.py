"""Race node: owns racing.race.RaceStack (PF localization, Pure Pursuit, fallback, laps).

Sub  /scan              sensor_msgs/LaserScan      (rplidar_ros)
     /wheel_state       sensor_msgs/JointState     (esp32_bridge)
     /imu               sensor_msgs/Imu            (esp32_bridge; drives odometry updates)
     /start_light/go    std_msgs/Bool              (vision_node)
     /aruco/fix         geometry_msgs/PoseStamped  (vision_node, map frame)
     /initialpose       geometry_msgs/PoseWithCovarianceStamped (rviz "2D Pose Estimate")
Pub  /drive             ackermann_msgs/AckermannDriveStamped
     /pf/pose           geometry_msgs/PoseWithCovarianceStamped
     /race/status       std_msgs/String (JSON)
     TF map -> base_link   (base_link = rear axle centre)
Srv  ~/start  (std_srvs/Trigger)  manual GO, bench tests only
     ~/reset  (std_srvs/Trigger)  re-initialise at the grid slot, back to WAIT_START
"""
import json
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from racing.control import VehicleParams
from racing.localization import PFConfig, deskew
from racing.planning import SpeedLimits, bump_zone
from racing.race import Mode, Phase, RaceConfig, RaceStack
from racing.scan import scan_arrays
from racing.track import OFFICIAL_DIR, OccupancyMap, grid_slot_pose, load_centerline, load_scene


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RaceNode(Node):
    def __init__(self):
        super().__init__("race_node")
        P = self.declare_parameter
        self.slot = P("grid_slot", 0).value
        laps = P("laps", 5).value
        a_lat_g = P("a_lat_g", 0.45).value
        v_max = P("v_max", 2.0).value
        lane_change = P("lane_change", False).value
        self.lidar_yaw = P("lidar_yaw", 0.0).value
        self.lidar_flip = P("lidar_flip", False).value
        lidar_x = P("lidar_x", 0.10).value
        wheelbase = P("wheelbase", 0.145).value
        max_steer = math.radians(P("max_steer_deg", 30.0).value)
        self.deskew_on = P("deskew", True).value
        self.control_hz = P("control_hz", 50.0).value

        scene = load_scene()
        params = VehicleParams(wheelbase=wheelbase, max_steer=max_steer)
        limits = SpeedLimits(a_lat=a_lat_g * 9.81, v_max=v_max, zones=bump_zone(scene))
        cfg = RaceConfig(laps=laps, limits=limits, lane_change_enabled=lane_change)
        self.scene = scene
        self.params = params
        self.stack = RaceStack(load_centerline(), OccupancyMap.load(OFFICIAL_DIR / "map.yaml"), scene,
                               params, cfg, PFConfig(lidar_x=lidar_x))
        self.reset_to_slot()

        self.pub_drive = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "pf/pose", 10)
        self.pub_status = self.create_publisher(String, "race/status", 5)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, "wheel_state", self.on_wheels, 50)
        self.create_subscription(Imu, "imu", self.on_imu, 50)
        self.create_subscription(Bool, "start_light/go", self.on_go, 10)
        self.create_subscription(PoseStamped, "aruco/fix", self.on_fix, 10)
        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.on_initialpose, 5)
        self.create_service(Trigger, "~/start", self.srv_start)
        self.create_service(Trigger, "~/reset", self.srv_reset)
        self.create_timer(1.0 / self.control_hz, self.on_control)
        self.create_timer(0.2, self.on_status)

        self._wheels = (0.0, 0.0)
        self._imu_t = None
        self._last_phase = None
        self.get_logger().info(f"race node ready, grid slot {self.slot}, waiting for start light")

    # ------------------------------------------------------------ setup
    def reset_to_slot(self):
        x, y, yaw = grid_slot_pose(self.slot, self.scene)  # body centre -> rear axle
        x -= math.cos(yaw) * self.params.wheelbase / 2
        y -= math.sin(yaw) * self.params.wheelbase / 2
        self.stack.reset_pose(x, y, yaw)
        self.stack.phase = Phase.WAIT_START

    # ------------------------------------------------------------ inputs
    def on_wheels(self, msg: JointState):
        if len(msg.velocity) >= 2:
            self._wheels = (msg.velocity[0], msg.velocity[1])

    def on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.01 if self._imu_t is None else min(max(t - self._imu_t, 0.0), 0.1)
        self._imu_t = t
        self.stack.on_odom(self._wheels[0], self._wheels[1], msg.angular_velocity.z, dt)

    def on_scan(self, msg: LaserScan):
        a, r, bdt = scan_arrays(msg.angle_min, msg.angle_increment, msg.ranges, self.lidar_yaw,
                                msg.time_increment, msg.scan_time, self.lidar_flip)
        r = np.where((r < msg.range_min) | (r > msg.range_max), np.inf, r)
        if self.deskew_on and np.any(bdt):
            ok = np.isfinite(r)
            a2, r2 = deskew(a[ok], r[ok], bdt[ok], self.stack.odom.v, self.stack.odom.yaw_rate)
            a, r = a2, r2
        self.stack.on_scan(a, r)
        self.publish_pose(msg.header.stamp)

    def on_go(self, msg: Bool):
        if msg.data:
            self.stack.on_start_light(True)

    def on_fix(self, msg: PoseStamped):
        self.stack.on_absolute_fix(msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.stack.reset_pose(p.position.x, p.position.y, yaw_of(p.orientation), std_xy=0.1)
        self.get_logger().info(f"re-initialised at ({p.position.x:.2f}, {p.position.y:.2f})")

    def srv_start(self, req, res):
        self.stack.on_start_light(True)
        res.success, res.message = True, "GO (manual)"
        return res

    def srv_reset(self, req, res):
        self.reset_to_slot()
        res.success, res.message = True, f"reset to grid slot {self.slot}"
        return res

    # ------------------------------------------------------------ outputs
    def on_control(self):
        steer, v = self.stack.command()
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.steering_angle = float(steer)
        m.drive.speed = float(v)
        self.pub_drive.publish(m)

    def publish_pose(self, stamp):
        x, y, yaw = self.stack.pose()
        sxy, syaw = self.stack.pf.spread()
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = sxy**2 / 2
        cov[35] = syaw**2
        msg.pose.covariance = cov
        self.pub_pose.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)

    def on_status(self):
        s = self.stack
        if s.phase != self._last_phase:
            self.get_logger().info(f"phase -> {s.phase.value}")
            self._last_phase = s.phase
        sxy, _ = s.pf.spread()
        self.pub_status.publish(String(data=json.dumps({
            "phase": s.phase.value, "mode": s.mode.value, "laps": s.laps_done,
            "lap_times": [round(t, 2) for t in s.lap_times], "spread_cm": round(sxy * 100, 1),
            "scan_score": round(s.pf.last_score, 2), "wheel_scale": round(s.wheel_scale, 3),
            "lane": s.lane, "obstacles": len(s._obstacles),
            "fallback": s.mode is Mode.FALLBACK})))


def main():
    rclpy.init()
    node = RaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

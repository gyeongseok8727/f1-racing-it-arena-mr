"""ESP32 <-> ROS bridge (docs/PROTOCOL.md).

Sub  /drive         ackermann_msgs/AckermannDriveStamped   (speed m/s, steering_angle rad)
Pub  /wheel_state   sensor_msgs/JointState  velocity = [v_left, v_right] m/s
     /imu           sensor_msgs/Imu         angular_velocity.z (falls back to wheel yaw rate)
     /odom          nav_msgs/Odometry       dead-reckoning, for rviz/debug only
     /vehicle/status std_msgs/String        JSON: armed, faults, battery
"""
import json
import math
import threading
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry as OdomMsg
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

from racing.localization import Odometry
from racing.protocol import Command, FrameParser, State, T_STATE

TRACK_WIDTH = 0.13


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__("esp32_bridge")
        self.port = self.declare_parameter("port", "/dev/ttyUSB0").value
        self.baud = self.declare_parameter("baud", 921600).value
        self.ediff_gain = self.declare_parameter("ediff_gain", 0.7).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.2).value
        self.max_speed = self.declare_parameter("max_speed", 3.5).value

        self.pub_wheels = self.create_publisher(JointState, "wheel_state", 50)
        self.pub_imu = self.create_publisher(Imu, "imu", 50)
        self.pub_odom = self.create_publisher(OdomMsg, "odom", 20)
        self.pub_status = self.create_publisher(String, "vehicle/status", 5)
        self.create_subscription(AckermannDriveStamped, "drive", self.on_drive, 10)

        self._drive = (0.0, 0.0)
        self._drive_t = 0.0
        self._seq = 0
        self._odom = Odometry()
        self._last_state_t = None
        self._status_tick = 0

        import serial

        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        self.lock = threading.Lock()
        threading.Thread(target=self.reader, daemon=True).start()
        self.create_timer(0.02, self.send_cmd)  # 50 Hz
        self.get_logger().info(f"ESP32 bridge on {self.port} @ {self.baud}")

    def on_drive(self, msg: AckermannDriveStamped):
        v = max(-self.max_speed, min(self.max_speed, msg.drive.speed))
        self._drive = (v, msg.drive.steering_angle)
        self._drive_t = time.monotonic()

    def send_cmd(self):
        fresh = time.monotonic() - self._drive_t < self.cmd_timeout
        v, steer = self._drive if fresh else (0.0, self._drive[1])
        self._seq = (self._seq + 1) & 0xFFFF
        frame = Command(self._seq, v, steer, drive_enable=fresh, ediff_gain=self.ediff_gain).encode()
        with self.lock:
            self.ser.write(frame)

    def reader(self):
        parser = FrameParser()
        while rclpy.ok():
            data = self.ser.read(256)
            if not data:
                continue
            for ftype, payload in parser.feed(data):
                if ftype == T_STATE and len(payload) == 20:
                    self.on_state(State.decode(payload))

    def on_state(self, st: State):
        now = self.get_clock().now().to_msg()
        imu_ok = "imu" not in st.fault_names()
        gz = st.gyro_z if imu_ok else (st.v_right - st.v_left) / TRACK_WIDTH

        js = JointState()
        js.header.stamp = now
        js.name = ["rear_left", "rear_right"]
        js.velocity = [st.v_left, st.v_right]
        self.pub_wheels.publish(js)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = gz
        imu.angular_velocity_covariance[8] = 0.02**2 if imu_ok else 0.2**2
        self.pub_imu.publish(imu)

        t = st.t_ms / 1000.0
        dt = 0.01 if self._last_state_t is None else max(0.0, min(0.1, t - self._last_state_t))
        self._last_state_t = t
        self._odom.update(st.v_left, st.v_right, gz, dt)
        od = OdomMsg()
        od.header.stamp = now
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = self._odom.x, self._odom.y
        od.pose.pose.orientation.z = math.sin(self._odom.yaw / 2)
        od.pose.pose.orientation.w = math.cos(self._odom.yaw / 2)
        od.twist.twist.linear.x = self._odom.v
        od.twist.twist.angular.z = gz
        self.pub_odom.publish(od)

        self._status_tick += 1
        if self._status_tick % 10 == 0:  # 10 Hz
            self.pub_status.publish(String(data=json.dumps({
                "armed": st.armed, "faults": st.fault_names(), "battery_v": round(st.batt_v, 2),
                "steer": round(st.steer, 3), "cmd_seq_echo": st.cmd_seq, "esp_ms": st.t_ms})))


def main():
    rclpy.init()
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(Command(0, 0.0, 0.0, drive_enable=False).encode())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

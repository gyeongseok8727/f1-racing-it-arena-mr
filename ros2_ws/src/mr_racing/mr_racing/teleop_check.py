"""Bring-up helper: publish a fixed /drive command for a few seconds, then stop.

Use with esp32_bridge only (not race_node) for calibration on blocks or the testbed:
  ros2 run mr_racing teleop_check --ros-args -p speed:=0.5 -p steer_deg:=0.0 -p duration:=3.0
  - wheel scale: drive straight a known distance, compare with /odom
  - steering map: steer_deg sweep, measure the wheel angle / turning circle
"""
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("teleop_check")
    speed = node.declare_parameter("speed", 0.3).value
    steer = math.radians(node.declare_parameter("steer_deg", 0.0).value)
    duration = node.declare_parameter("duration", 2.0).value
    pub = node.create_publisher(AckermannDriveStamped, "drive", 10)
    t0 = time.monotonic()
    node.get_logger().info(f"driving {speed} m/s, {math.degrees(steer):.1f} deg for {duration} s")
    try:
        while time.monotonic() - t0 < duration:
            m = AckermannDriveStamped()
            m.drive.speed, m.drive.steering_angle = float(speed), float(steer)
            pub.publish(m)
            time.sleep(0.02)
    finally:
        for _ in range(10):
            pub.publish(AckermannDriveStamped())
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

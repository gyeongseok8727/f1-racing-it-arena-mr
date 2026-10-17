"""Full race stack. Bench tests: `use_lidar:=false use_vision:=false`."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("mr_racing"), "config", "race.yaml")
    params = LaunchConfiguration("params")
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=cfg),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_vision", default_value="true"),
        DeclareLaunchArgument("use_race", default_value="true"),
        Node(package="rplidar_ros", executable="rplidar_node", name="rplidar_node",
             parameters=[params], condition=IfCondition(LaunchConfiguration("use_lidar"))),
        Node(package="mr_racing", executable="esp32_bridge", name="esp32_bridge",
             parameters=[params], output="screen"),
        Node(package="mr_racing", executable="race_node", name="race_node",
             parameters=[params], output="screen",
             condition=IfCondition(LaunchConfiguration("use_race"))),
        Node(package="mr_racing", executable="vision_node", name="vision_node",
             parameters=[params], output="screen",
             condition=IfCondition(LaunchConfiguration("use_vision"))),
    ])

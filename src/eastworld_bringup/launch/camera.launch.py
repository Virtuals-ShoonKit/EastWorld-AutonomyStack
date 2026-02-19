#!/usr/bin/python3
"""
CSI Camera Streamer — standalone launch.

Launches the Jetson CSI camera with HW H.264/JPEG encoding for
Foxglove Studio streaming over WiFi.

Usage:
  ros2 launch eastworld_bringup camera.launch.py
  ros2 launch eastworld_bringup camera.launch.py encoding:=jpeg
  ros2 launch eastworld_bringup camera.launch.py width:=1920 height:=1080

Requires:
  - Jetson with CSI camera connected
  - nvargus-daemon running
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"
os.environ["ROS_DOMAIN_ID"] = "42"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"


def generate_launch_description():

    eastworld_camera_dir = get_package_share_directory("eastworld_camera")
    csi_camera_config = os.path.join(
        eastworld_camera_dir, "config", "csi_streamer.yaml"
    )

    # ── Launch arguments ───────────────────────────────────────────────
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level (debug, info, warn, error)",
    )

    # ── CSI Camera streamer ───────────────────────────────────────────
    csi_camera = Node(
        package="eastworld_camera",
        executable="csi_streamer",
        name="csi_streamer",
        output="screen",
        parameters=[csi_camera_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription([
        log_level_arg,
        csi_camera,
    ])

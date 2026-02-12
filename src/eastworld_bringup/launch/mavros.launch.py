#!/usr/bin/python3
"""
MAVROS bridge launch file for PX4 over UART.

Connects to PX4 flight controller via /dev/ttyTHS1 at 921600 baud.
Receives IMU-propagated odometry from FAST-LIVO2 on /mavros/odometry/out
and forwards it to PX4 EKF2 for external vision fusion.

Usage:
  ros2 launch eastworld_bringup mavros.launch.py

Override FCU URL:
  ros2 launch eastworld_bringup mavros.launch.py fcu_url:=/dev/ttyTHS1:921600
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():

    pkg_dir = get_package_share_directory("eastworld_bringup")
    pluginlists_yaml = os.path.join(pkg_dir, "config", "px4_pluginlists.yaml")
    config_yaml = os.path.join(pkg_dir, "config", "px4_config.yaml")

    # --------------- Launch arguments ---------------
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyTHS1:921600",
        description="FCU connection URL  (serial: /dev/ttyTHS1:921600, udp: udp://:14540@)",
    )

    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="udp://:14555@127.0.0.1:14550",
        description="GCS bridge URL (QGC on localhost)",
    )

    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system",
        default_value="1",
        description="MAVLink target system id",
    )

    respawn_arg = DeclareLaunchArgument(
        "respawn",
        default_value="true",
        description="Respawn MAVROS if it crashes",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="warn",
        description="MAVROS log level (debug, info, warn, error)",
    )

    # --------------- MAVROS node ---------------
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        output="screen",
        respawn=LaunchConfiguration("respawn"),
        parameters=[
            pluginlists_yaml,
            config_yaml,
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "tgt_system": LaunchConfiguration("tgt_system"),
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        respawn_arg,
        log_level_arg,
        mavros_node,
    ])

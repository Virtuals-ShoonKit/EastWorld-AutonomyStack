#!/usr/bin/python3
"""
Camera Tilt Servo Controller — standalone launch.

Controls a PX4 Peripheral Actuator Set 1 output via MAVROS
DO_SET_ACTUATOR to tilt the camera between three preset positions:
forward, half, and down.

Requires MAVROS to be running (provides /mavros/cmd/command service).

Usage:
  ros2 launch eastworld_bringup camera_tilt.launch.py
  ros2 launch eastworld_bringup camera_tilt.launch.py actuator_index:=2

Trigger from CLI:
  ros2 service call /camera_tilt/set_forward std_srvs/srv/Trigger
  ros2 service call /camera_tilt/set_half    std_srvs/srv/Trigger
  ros2 service call /camera_tilt/set_down    std_srvs/srv/Trigger
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
    camera_tilt_config = os.path.join(
        eastworld_camera_dir, "config", "camera_tilt.yaml"
    )

    # ── Launch arguments ───────────────────────────────────────────────
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level (debug, info, warn, error)",
    )

    actuator_index_arg = DeclareLaunchArgument(
        "actuator_index",
        default_value="1",
        description="Actuator index within Peripheral Actuator Set 1 (1-6)",
    )

    value_forward_arg = DeclareLaunchArgument(
        "value_forward",
        default_value="-1.0",
        description="Normalized value for forward position (-1.0 to 1.0)",
    )

    value_half_arg = DeclareLaunchArgument(
        "value_half",
        default_value="0.0",
        description="Normalized value for half-tilt position (-1.0 to 1.0)",
    )

    value_down_arg = DeclareLaunchArgument(
        "value_down",
        default_value="1.0",
        description="Normalized value for full-down position (-1.0 to 1.0)",
    )

    # ── Camera tilt servo controller ──────────────────────────────────
    camera_tilt = Node(
        package="eastworld_camera",
        executable="camera_tilt",
        name="camera_tilt",
        output="screen",
        parameters=[
            camera_tilt_config,
            {
                "actuator_index": LaunchConfiguration("actuator_index"),
                "value_forward": LaunchConfiguration("value_forward"),
                "value_half": LaunchConfiguration("value_half"),
                "value_down": LaunchConfiguration("value_down"),
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription([
        log_level_arg,
        actuator_index_arg,
        value_forward_arg,
        value_half_arg,
        value_down_arg,
        camera_tilt,
    ])

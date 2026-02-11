#!/usr/bin/python3
"""
EastWorld AutonomyStack bringup launch composition.

Launches the full flight stack in one command:
  1. Livox MID360 driver  (LiDAR + IMU @ 20 Hz)
  2. FAST-LIVO2 LIO-only  (pose estimation, publishes /mavros/vision_pose/pose)
  3. MAVROS bridge         (PX4 <-> ROS2 over /dev/ttyTHS1:921600)
  4. RViz2 (optional)

Usage:
  ros2 launch eastworld_bringup bringup.launch.py
  ros2 launch eastworld_bringup bringup.launch.py use_rviz:=false
  ros2 launch eastworld_bringup bringup.launch.py fcu_url:=udp://:14540@
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────────
    bringup_dir = get_package_share_directory("eastworld_bringup")
    fast_livo_dir = get_package_share_directory("fast_livo")
    livox_dir = get_package_share_directory("livox_ros_driver2")

    # ── Config file paths ──────────────────────────────────────────────
    mid360_config = os.path.join(bringup_dir, "config", "mid360.yaml")
    pluginlists_yaml = os.path.join(bringup_dir, "config", "px4_pluginlists.yaml")
    config_yaml = os.path.join(bringup_dir, "config", "px4_config.yaml")
    camera_config = os.path.join(fast_livo_dir, "config", "camera_pinhole.yaml")
    rviz_config = os.path.join(fast_livo_dir, "rviz_cfg", "fast_livo2.rviz")
    livox_json = os.path.join(livox_dir, "config", "MID360_config.json")

    # ── Launch arguments ───────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for visualization",
    )

    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyTHS1:921600",
        description="FCU connection URL (serial or UDP)",
    )

    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="udp://:14550@127.0.0.1:14550",
        description="GCS bridge URL (QGC on localhost)",
    )

    # ── 1. Livox MID360 driver ─────────────────────────────────────────
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 20.0},
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"user_config_path": livox_json},
            {"cmdline_input_bd_code": "47MCNB700331612"},
        ],
    )

    # ── 2. FAST-LIVO2 (LIO-only, no camera) ───────────────────────────
    fast_livo2 = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        output="screen",
        respawn=True,
        parameters=[
            mid360_config,
            {"camera_config": camera_config},
        ],
    )

    # ── 3. MAVROS bridge ──────────────────────────────────────────────
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        respawn=True,
        parameters=[
            pluginlists_yaml,
            config_yaml,
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # ── 4. RViz2 (optional) ───────────────────────────────────────────
    rviz2 = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # ── Compose ───────────────────────────────────────────────────────
    return LaunchDescription([
        use_rviz_arg,
        fcu_url_arg,
        gcs_url_arg,
        livox_driver,
        fast_livo2,
        mavros_node,
        rviz2,
    ])

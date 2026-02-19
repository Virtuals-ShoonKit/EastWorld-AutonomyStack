#!/usr/bin/python3
"""
EastWorld AutonomyStack bringup launch composition.

Launches the full flight stack in one command:
  1. Livox MID360 driver  (LiDAR + IMU @ 20 Hz)
  2. FAST-LIVO2 LIO-only  (pose estimation, IMU-propagated odom -> /mavros/odometry/out @ 50 Hz)
  3. MAVROS bridge         (PX4 <-> ROS2 over /dev/ttyTHS1:921600)
  4. Foxglove Bridge       (WebSocket server for Foxglove Studio, default port 8765)
  5. CSI Camera streamer   (HW H.264 for Foxglove, optional, use_camera:=true)
  6. RViz2 (optional)

Usage:
  ros2 launch eastworld_bringup bringup.launch.py
  ros2 launch eastworld_bringup bringup.launch.py use_rviz:=false
  ros2 launch eastworld_bringup bringup.launch.py fcu_url:=udp://:14540@
"""

import math
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"
os.environ["ROS_DOMAIN_ID"] = "42"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────────
    bringup_dir = get_package_share_directory("eastworld_bringup")
    fast_livo_dir = get_package_share_directory("fast_livo")
    livox_dir = get_package_share_directory("livox_ros_driver2")

    # ── Config file paths ──────────────────────────────────────────────
    mid360_config = os.path.join(bringup_dir, "config", "mid360.yaml")
    cyclonedds_xml = os.path.join(bringup_dir, "config", "cyclonedds.xml")
    os.environ.setdefault("CYCLONEDDS_URI", f"file://{cyclonedds_xml}")
    pluginlists_yaml = os.path.join(bringup_dir, "config", "px4_pluginlists.yaml")
    config_yaml = os.path.join(bringup_dir, "config", "px4_config.yaml")
    rviz_config = os.path.join(bringup_dir, "config", "fast-livo.rviz")
    camera_config = os.path.join(fast_livo_dir, "config", "camera_pinhole.yaml")
    livox_json = os.path.join(livox_dir, "config", "MID360_config.json")

    # ── Launch arguments ───────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 for visualization",
    )

    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyTHS1:921600",
        description="FCU connection URL (serial or UDP)",
    )

    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="udp://:14555@127.0.0.1:14550",
        description="GCS bridge URL (QGC on localhost)",
    )

    stack_log_level_arg = DeclareLaunchArgument(
        "stack_log_level",
        default_value="warn",
        description="Log level for Livox, FAST-LIVO, MAVROS",
    )

    rviz_log_level_arg = DeclareLaunchArgument(
        "rviz_log_level",
        default_value="error",
        description="RViz log level",
    )

    use_map_camera_tf_arg = DeclareLaunchArgument(
        "use_map_camera_tf",
        default_value="true",
        description="Publish static TFs: map->camera_init, map->odom",
    )

    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Parent frame for static TF",
    )

    camera_init_frame_arg = DeclareLaunchArgument(
        "camera_init_frame",
        default_value="camera_init",
        description="Child frame for static TF",
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Launch Foxglove Bridge WebSocket server",
    )

    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="Foxglove Bridge WebSocket port",
    )

    use_camera_arg = DeclareLaunchArgument(
        "use_camera",
        default_value="true",
        description="Launch CSI camera streamer (H.264 to Foxglove)",
    )

    use_camera_tilt_arg = DeclareLaunchArgument(
        "use_camera_tilt",
        default_value="true",
        description="Launch camera tilt servo controller",
    )

    # ── 1. Livox MID360 driver ─────────────────────────────────────────
    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("stack_log_level")],
        parameters=[
            {"xfer_format": 1},  # 0=PointCloud2, 1=CustomMsg (required by FAST-LIVO lidar_type 1)
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
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
        arguments=["--ros-args", "--log-level", LaunchConfiguration("stack_log_level")],
        parameters=[
            mid360_config,
            {"camera_config": camera_config},
        ],
    )

    # ── 3. MAVROS bridge ──────────────────────────────────────────────
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
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
        arguments=["--ros-args", "--log-level", LaunchConfiguration("stack_log_level")],
    )

    map_to_camera_init_tf = Node(
        condition=IfCondition(LaunchConfiguration("use_map_camera_tf")),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_camera_init_tf",
        output="screen",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            LaunchConfiguration("map_frame"),
            LaunchConfiguration("camera_init_frame"),
            "--ros-args", "--log-level", LaunchConfiguration("stack_log_level"),
        ],
    )

    map_to_odom_tf = Node(
        condition=IfCondition(LaunchConfiguration("use_map_camera_tf")),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        output="screen",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "map",
            "odom",
            "--ros-args", "--log-level", LaunchConfiguration("stack_log_level"),
        ],
    )

    # Static TF: imu_link -> base_link  (undo sensor forward tilt)
    # LiDAR/IMU is pitched 22° nose-down from body level.
    # Negate to cancel the tilt: Ry(-22°) brings sensor frame back to body level.
    SENSOR_PITCH_DEG = 22.0
    pitch_rad = -SENSOR_PITCH_DEG * math.pi / 180.0

    imu_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_to_base_link_tf",
        output="screen",
        arguments=[
            "0", "0", "0",                          # x y z  (no lever-arm offset)
            "0", str(pitch_rad), "0",                # yaw pitch roll  (radians)
            "imu_link",
            "base_link",
            "--ros-args", "--log-level", LaunchConfiguration("stack_log_level"),
        ],
    )

    # ── 4. Foxglove Bridge ─────────────────────────────────────────────
    foxglove_bridge = Node(
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("foxglove_port")},
            {"address": "0.0.0.0"},
            {"send_buffer_limit": 2000000},
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("stack_log_level")],
    )

    # ── 5. CSI Camera streamer (optional) ─────────────────────────────
    eastworld_camera_dir = get_package_share_directory("eastworld_camera")
    csi_camera_config = os.path.join(eastworld_camera_dir, "config", "csi_streamer.yaml")

    csi_camera = Node(
        condition=IfCondition(LaunchConfiguration("use_camera")),
        package="eastworld_camera",
        executable="csi_streamer",
        name="csi_streamer",
        output="screen",
        parameters=[csi_camera_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("stack_log_level")],
    )

    # ── 5b. Camera tilt servo controller (optional) ─────────────────
    camera_tilt_config = os.path.join(
        eastworld_camera_dir, "config", "camera_tilt.yaml"
    )

    camera_tilt = Node(
        condition=IfCondition(LaunchConfiguration("use_camera_tilt")),
        package="eastworld_camera",
        executable="camera_tilt",
        name="camera_tilt",
        output="screen",
        parameters=[camera_tilt_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("stack_log_level")],
    )

    # ── 6. RViz2 (optional) ───────────────────────────────────────────
    rviz2 = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", rviz_config,
            "--ros-args", "--log-level", LaunchConfiguration("rviz_log_level"),
        ],
    )

    # ── Compose ───────────────────────────────────────────────────────
    return LaunchDescription([
        use_rviz_arg,
        fcu_url_arg,
        gcs_url_arg,
        stack_log_level_arg,
        rviz_log_level_arg,
        use_map_camera_tf_arg,
        map_frame_arg,
        camera_init_frame_arg,
        use_foxglove_arg,
        foxglove_port_arg,
        use_camera_arg,
        use_camera_tilt_arg,
        livox_driver,
        fast_livo2,
        mavros_node,
        map_to_camera_init_tf,
        map_to_odom_tf,
        imu_to_base_link_tf,
        foxglove_bridge,
        csi_camera,
        camera_tilt,
        rviz2,
    ])

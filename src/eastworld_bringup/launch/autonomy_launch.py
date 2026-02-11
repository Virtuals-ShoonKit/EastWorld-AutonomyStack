#!/usr/bin/env python3
"""
Launch Livox Mid-360 driver and SPARK-FAST-LIO using configs from eastworld_bringup/config/.
Keeps your configs and launch logic separate from upstream livox_ros_driver2 and spark-fast-lio.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    bringup_share = get_package_share_directory('eastworld_bringup')
    config_dir = os.path.join(bringup_share, 'config')

    # Livox driver: use MID360_config.json from this package's config/
    livox_config = os.path.join(config_dir, 'MID360_config.json')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/.ros/livox_test.lvx'},
            {'user_config_path': livox_config},
            {'cmdline_input_bd_code': '47MCNB700331612'},
        ],
    )

    # SPARK-FAST-LIO: use config file from this package's config/ (e.g. mid360.yaml)
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='mid360.yaml',
        description='SPARK-FAST-LIO config filename in eastworld_bringup/config/',
    )
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Run RViz2')
    use_rviz = LaunchConfiguration('rviz')

    config_path_param = PathJoinSubstitution([TextSubstitution(text=config_dir), config_file])
    spark_lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        parameters=[config_path_param, {'use_sim_time': use_sim_time}],
        remappings=[
            ('lidar', '/livox/lidar'),
            ('imu', '/livox/imu'),
        ],
        output='screen',
    )

    # Static TF: base_link -> lidar (identity — adjust if lidar is offset from body origin)
    # Required by SPARK-FAST-LIO when base_frame is set (gravity alignment uses this).
    static_tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('spark_fast_lio'), 'rviz', 'velodyne_mit.rviz')],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        use_rviz_arg,
        static_tf_base_lidar,
        livox_driver,
        spark_lio_node,
        rviz_node,
    ])

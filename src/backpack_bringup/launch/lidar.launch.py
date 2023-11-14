#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

from backpack_bringup.helpers import run_sensors_command


def generate_launch_description():
    ld = LaunchDescription()

    run_sensors = run_sensors_command(ld)

    # LDROBOT LiDAR publisher node
    lidar_driver_node = Node(
        condition=IfCondition(run_sensors),
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='stl27l_lidar_driver',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_STL27L'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyAMA2'},
            {'port_baudrate': 921600},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 0.0},
            {'angle_crop_max': 0.0}
        ]
    )
    ld.add_action(lidar_driver_node)

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_stl27l',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser']
    )
    ld.add_action(base_link_to_laser_tf_node)

    return ld

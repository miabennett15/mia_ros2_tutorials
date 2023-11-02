#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='STL27L',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_STL27L'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 921600},
            {'laser_scan_dir': False},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 0.0},
            {'angle_crop_max': 0.0}
        ]
    )

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_stl27l',
        arguments=['0','0','0.18','0','0','0','base_link','base_laser']
    )

    # Bosch BNO055 Inertial Measurement Unit (IMU) node
    imu_config = os.path.join(
        get_package_share_directory('backpack_bringup'),
        'launch',
        'bno055_params.yaml'
    )
    imu_driver = Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [imu_config]
    )

    # Add the LiDAR nodes
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)

    # Add the IMU node
    ld.add_action(imu_driver)

    return ld

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

from backpack_bringup.helpers import get_config_params, run_sensors_command


def generate_launch_description():
    ld = LaunchDescription()

    run_sensors = run_sensors_command(ld)
    config = get_config_params()

    # Bosch BNO055 Inertial Measurement Unit (IMU) node
    imu_driver_node = Node(
        condition=IfCondition(run_sensors),
        package='bno055',
        executable='bno055',
        name='bno055_imu_driver',
        parameters=[config],
        remappings=[
            ('/bno055/imu_raw', '/imu/data_raw'),
            ('/bno055/imu', '/imu/data'),
        ]
    )
    ld.add_action(imu_driver_node)

    # Add transform to IMU frame
    static_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0',
                '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link_oriented'
            ]
        )
    ld.add_action(static_transform)

    # IMU transformer
    imu_transformer = Node(
        package='imu_tf',
        executable='transform',
    )
    ld.add_action(imu_transformer)

    return ld

#!/usr/bin/env python3

from launch import LaunchDescription

from backpack_bringup.helpers import launch_file_nodes


def generate_launch_description():
    return LaunchDescription([
        launch_file_nodes('/foxglove.launch.py'),
        launch_file_nodes('/gps.launch.py'),
        launch_file_nodes('/imu.launch.py'),
        launch_file_nodes('/lidar.launch.py'),
    ])

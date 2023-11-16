#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

from backpack_bringup.helpers import launch_file_nodes


def generate_launch_description():
    # System stats node is a special case that doesn't warrant it's own launch file
    system_stats_node = Node(
        package='system_stats_pkg',
        executable='system_stats'
    )

    return LaunchDescription([
        launch_file_nodes('/foxglove.launch.py'),
        launch_file_nodes('/gps.launch.py'),
        launch_file_nodes('/imu.launch.py'),
        launch_file_nodes('/lidar.launch.py'),
        system_stats_node
    ])

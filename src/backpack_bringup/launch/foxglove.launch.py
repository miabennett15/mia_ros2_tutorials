#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Foxglove bridge node
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge'
    )
    ld.add_action(foxglove_node)

    return ld

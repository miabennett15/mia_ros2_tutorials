#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

from backpack_bringup.helpers import get_config_params, run_sensors_command


def generate_launch_description():
    ld = LaunchDescription()

    run_sensors = run_sensors_command(ld)
    config = get_config_params()

    # GPS Module Driver
    gps_driver_node = Node(
        condition=IfCondition(run_sensors),
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[config]
    )
    ld.add_action(gps_driver_node)

    return ld

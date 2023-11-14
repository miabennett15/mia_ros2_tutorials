#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def get_config_params():
    return os.path.join(
        get_package_share_directory('backpack_bringup'),
        'config',
        'params.yaml'
    )


def launch_file_nodes(filename):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('backpack_bringup'), 'launch'),
            filename
        ])
    )


def run_sensors_command(ld):
    run_sensors = LaunchConfiguration('run_sensors')

    # Declare the run_sensors launch argument and add it to the LaunchDescription
    declare_run_sensors_cmd = DeclareLaunchArgument(
        'run_sensors',
        default_value='True',
        description='Whether to start the robot state publisher')
    ld.add_action(declare_run_sensors_cmd)

    return run_sensors

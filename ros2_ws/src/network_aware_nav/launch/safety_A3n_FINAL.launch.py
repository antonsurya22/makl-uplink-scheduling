#!/usr/bin/env python3

import os

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'network_aware_nav'
    )

    params_file = os.path.join(
        pkg_share,
        'config',
        'safety_A3n_FINAL.yaml',
    )

    robot_state_monitor = Node(
        package='network_aware_nav',
        executable='robot_state_monitor',
        name='robot_state_monitor',
        output='screen',
        parameters=[params_file],
    )

    safety_supervisor = Node(
        package='network_aware_nav',
        executable='safety_supervisor',
        name='safety_supervisor',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        robot_state_monitor,
        safety_supervisor,
    ])

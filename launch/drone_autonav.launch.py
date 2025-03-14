#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_autonav',
            executable='occupancy_mapper',
            name='occupancy_mapper'
        ),
        Node(
            package='drone_autonav',
            executable='path_planner',
            name='path_planner'
        ),
        Node(
            package='drone_autonav',
            executable='trajectory_controller',
            name='trajectory_controller'
        ),
    ])

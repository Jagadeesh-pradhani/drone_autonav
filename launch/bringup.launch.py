#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap, Node


def generate_launch_description():
    curr_pkg = get_package_share_directory('drone_autonav')
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')
    turtlebot3_launch_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    x_pose = DeclareLaunchArgument(
        "x_pose",
        default_value="1.0",
        description="Initial x position for the turtlebot in Gazebo"
    )
    y_pose = DeclareLaunchArgument(
        "y_pose",
        default_value="0.0",
        description="Initial y position for the turtlebot in Gazebo"
    )
    
    rviz_path = os.path.join(
        sjtu_drone_bringup_path, "rviz", "octamap.rviz"
    )
    

    
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        )
    
    uav_brigup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(curr_pkg, 'launch', 'uav.launch.py')
            )
        )
    
    ugv_rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_path, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
    
    ugv_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_launch_path, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose')
            }.items()
        )
    
    tf_ = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            output="screen"
        )
    
    tf_2 = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map_ugv"],
            output="screen"
        )
    
    tf_3 = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map_ugv"],
            output="screen"
        )
    
    uav_controller = Node(
            package='sjtu_drone_control',
            executable='fellow_path_controller',
            name='fellow_path_controller',
            output='screen',
        )
    
    octamap = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'octomap_online.launch.py')
            )
        )
    
    planner = Node(
            package='drone_autonav',
            executable='drone_autonav_node',
            name='drone_autonav_node',
            output='screen',
        )
    

    return LaunchDescription([
        declare_use_sim_time_cmd,
        x_pose,
        y_pose,
        rviz_node,
        uav_brigup,
        ugv_rsp,
        ugv_spawn,
        tf_,
        tf_2,
        # tf_3,
        uav_controller,
        octamap,
        planner
        
    ])

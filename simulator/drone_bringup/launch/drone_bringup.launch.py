#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    drone_bringup_path = get_package_share_directory('drone_bringup')



    rviz_path = os.path.join(
        drone_bringup_path, "rviz", "rviz.rviz"
    )

    # joystick_control_node = Node(
    #     package='drone_bringup',  
    #     executable='python3',  
    #     arguments=['joystick_control.py'],  0p
    #     output='screen',
    # )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            
        ),
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[
                {'enable_button': -1},
                {'require_enable_button': False},
                {'axis_linear.x': 0},
                {'axis_linear.y': 2},
                {'axis_linear.z': 6},
                {'axis_angular.pitch': 5},
                {'axis_angular.roll': 3},
                {'axis_angular.yaw': 7},
                {'scale_linear.x': 1.0},
                {'scale_linear.y': 1.0},
                {'scale_linear.z': 1.0},
                {'scale_angular.pitch': 1.0},
                {'scale_angular.roll': 1.0},
                {'scale_angular.yaw': 1.0}
            ]
        ),
        Node(
            package='drone_bringup',
            executable='joystick_control',
            name='joystick_controller',
            output='screen'
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", rviz_path
            ],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drone_bringup_path, 'launch', 'drone_gazebo.launch.py')
            )
        ),

        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            namespace="drone",
            output="screen",
            remappings = [
                (
                    'teleop_twist_keyboard', 'cmd_vel'
                )
            ]
        ),

    ])
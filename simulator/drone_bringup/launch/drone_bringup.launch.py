#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    drone_bringup_path = get_package_share_directory('drone_bringup')



    rviz_path = os.path.join(
        drone_bringup_path, "rviz", "rviz.rviz"
    )

    joystick_control_node = Node(
        package='simulator',  # Replace 'your_package_name' with the actual package name where 'joystick_control.py' is located
        executable='python3',  # Replace with 'python' if needed
        arguments=['joystick_control.py'],  # Reference the joystick script
        output='screen',
    )

    return LaunchDescription([
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
            prefix="xterm -e"
        ),

        joystick_control_node,

    ])
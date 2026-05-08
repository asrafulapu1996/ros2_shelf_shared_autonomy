#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device': '/dev/input/js1'}],
        output='screen'
    )

    gamepad_teleop_node = Node(
        package='mycobot_moveit_demos',
        executable='gamepad_teleop',
        name='gamepad_teleop',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([joy_node, gamepad_teleop_node])

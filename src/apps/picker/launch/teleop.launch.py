#!/usr/bin/env python3
"""
Terminal 2 — Teleoperation: gamepad driver + keyboard arm control

  source /opt/ros/jazzy/setup.bash && source ~/class_ws/install/setup.bash
  ros2 launch picker teleop.launch.py

Starts:
  - joy_node          : reads the gamepad (used by picker_node Start button)
  - keyboard_teleop   : move the arm manually with the keyboard (W/S/A/D/Q/E)

Run this AFTER sim.launch.py (Terminal 1) is fully up.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    pkg_moveit = FindPackageShare('mycobot_moveit_config').find('mycobot_moveit_config')
    config_path = os.path.join(pkg_moveit, 'config', robot_name)

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name='mycobot_moveit_config')
        .robot_description_semantic(
            file_path=os.path.join(config_path, f'{robot_name}.srdf'))
        .robot_description_kinematics(
            file_path=os.path.join(config_path, 'kinematics.yaml'))
        .joint_limits(
            file_path=os.path.join(config_path, 'joint_limits.yaml'))
        .trajectory_execution(
            file_path=os.path.join(config_path, 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    keyboard_teleop = Node(
        package='mycobot_moveit_demos',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    return [joy, keyboard_teleop]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='mycobot_280',
            description='Robot model name'),
        OpaqueFunction(function=launch_setup),
    ])

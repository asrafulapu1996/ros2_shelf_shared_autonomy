#!/usr/bin/env python3
"""
Terminal 1 — Simulation core: Gazebo + MoveIt 2 + RViz

Equivalent to running these two commands in sequence:
  ros2 launch mycobot_gazebo mycobot.gazebo.launch.py use_camera:=true
  ros2 launch mycobot_moveit_config move_group.launch.py   (after 20 s)

Usage:
  source /opt/ros/jazzy/setup.bash && cd ~/class_ws && source install/setup.bash
  ros2 launch picker sim.launch.py

Wait ~30 s for everything to be ready before starting other terminals.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_gazebo = FindPackageShare('mycobot_gazebo').find('mycobot_gazebo')
    pkg_moveit = FindPackageShare('mycobot_moveit_config').find('mycobot_moveit_config')

    # Exact equivalent of Terminal 1
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'mycobot.gazebo.launch.py')),
        launch_arguments={'use_camera': 'true'}.items(),
    )

    # Exact equivalent of Terminal 2, delayed until Gazebo + controllers are up
    moveit = TimerAction(
        period=30.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')),
        )],
    )

    return LaunchDescription([gazebo, moveit])

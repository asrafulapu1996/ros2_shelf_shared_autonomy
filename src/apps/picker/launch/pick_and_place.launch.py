#!/usr/bin/env python3
"""
Single-command launch — includes sim + teleop + pick in one terminal.

  source /opt/ros/jazzy/setup.bash && source ~/class_ws/install/setup.bash
  ros2 launch picker pick_and_place.launch.py

Or split across three terminals for easier monitoring:
  Terminal 1:  ros2 launch picker sim.launch.py
  Terminal 2:  ros2 launch picker teleop.launch.py
  Terminal 3:  ros2 launch picker pick.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration
    use_rviz = LaunchConfiguration('use_rviz').perform(context)

    pkg_picker = FindPackageShare('picker').find('picker')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_picker, 'launch', 'sim.launch.py')),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    teleop = TimerAction(
        period=35.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_picker, 'launch', 'teleop.launch.py')),
        )],
    )

    pick = TimerAction(
        period=42.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_picker, 'launch', 'pick.launch.py')),
        )],
    )

    return [sim, teleop, pick]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz with the MoveIt motion-planning panel'),
        OpaqueFunction(function=launch_setup),
    ])

#!/usr/bin/env python3
"""
Terminal 3 — Detection + autonomous pick-and-place

  source /opt/ros/jazzy/setup.bash && source ~/class_ws/install/setup.bash
  ros2 launch picker pick.launch.py

Starts:
  - medicine_detector : clusters RGBD point cloud, publishes /target_medicine_pose
  - picker_node       : listens on /joy Start button, executes pick-and-place cycle

Run this AFTER sim.launch.py (Terminal 1) is fully up.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    medicine_detector = Node(
        package='picker',
        executable='medicine_detector',
        name='medicine_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    picker_node = Node(
        package='picker',
        executable='picker_node',
        name='picker_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([medicine_detector, picker_node])

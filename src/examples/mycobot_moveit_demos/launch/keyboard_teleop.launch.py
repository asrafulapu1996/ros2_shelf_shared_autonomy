#!/usr/bin/env python3
"""
Launch the keyboard end-effector teleop node for the myCobot 280.

Run AFTER the Gazebo simulation + MoveIt move_group are already up:
  Terminal 1:  ros2 launch mycobot_gazebo mycobot.gazebo.launch.py world_file:=medicine_shelf_demo.world z:=0.52
  Terminal 2:  ros2 launch mycobot_moveit_config move_group.launch.py
  Terminal 3:  ros2 launch mycobot_moveit_demos keyboard_teleop.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_name = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_280',
        description='Name of the robot')

    def configure_setup(context):
        robot_name   = LaunchConfiguration('robot_name').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

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

        node = Node(
            package='mycobot_moveit_demos',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time == 'true'},
            ],
        )
        return [node]

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_name,
        OpaqueFunction(function=configure_setup),
    ])

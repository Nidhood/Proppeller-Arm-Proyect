#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------- LAUNCH DEPENDENCIES ----------------------
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------- LAUNCH DESCRIPTION ----------------------
def generate_launch_description():
    
    # Model xacro file path:
    xacro_file = PathJoinSubstitution([
        FindPackageShare('prop_arm_description'),
        'models',
        'prop_arm',
        'urdf',
        'prop_arm.urdf.xacro'
    ])
    
    # robot_description to launch the prop arm model:
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            "robot_description": ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str
            )
        }]
    )
    
    # Return the launch description:
    return LaunchDescription([
        robot_state_publisher_node,
    ])
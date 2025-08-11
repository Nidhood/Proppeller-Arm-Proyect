#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'gui_only',
            default_value='false',
            description='Launch only the GUI without the control system'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for the GUI node'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('prop_arm_gui'),
                'config',
                'gui_config.yaml'
            ]),
            description='Path to GUI configuration file'
        ),

        # Log info
        LogInfo(
            msg="Starting PropArm GUI..."
        ),

        # PropArm GUI Node
        Node(
            package='prop_arm_gui',
            executable='prop_arm_gui',
            name='prop_arm_gui_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            condition=UnlessCondition(LaunchConfiguration('gui_only'))
        ),

        # GUI-only mode (for standalone testing)
        Node(
            package='prop_arm_gui',
            executable='prop_arm_gui',
            name='prop_arm_gui_standalone',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'standalone_mode': True}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            condition=IfCondition(LaunchConfiguration('gui_only'))
        ),

        LogInfo(
            msg="PropArm GUI launched successfully"
        )
    ])
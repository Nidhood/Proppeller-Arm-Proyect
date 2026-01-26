#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------- LAUNCH DEPENDENCIES ----------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


# ---------------------- LAUNCH DESCRIPTION ----------------------
def generate_launch_description():
    config_file = LaunchConfiguration("config")
    limits_file = LaunchConfiguration("limits")

    default_config = PathJoinSubstitution([
        FindPackageShare("prop_arm_gui"),
        "config",
        "gui_config.yaml",
    ])
    
    default_limits = PathJoinSubstitution([
        FindPackageShare("prop_arm_gui"),
        "config",
        "limits_config.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the GUI YAML configuration file",
        ),
        
        DeclareLaunchArgument(
            "limits",
            default_value=default_limits,
            description="Path to the chart limits YAML configuration file",
        ),

        Node(
            package="prop_arm_gui",
            executable="prop_arm_gui_node",     
            name="prop_arm_gui_node",           
            output="screen",
            parameters=[config_file, limits_file],
        ),
    ])
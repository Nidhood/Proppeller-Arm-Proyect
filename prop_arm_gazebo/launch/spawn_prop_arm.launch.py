#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# ---------------------- LAUNCH DESCRIPTION ----------------------
def generate_launch_description():
    
    # Paths to included launch files
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("prop_arm_gazebo"), "launch", "start_world.launch.py"
    ])
    urdf_launch = PathJoinSubstitution([
        FindPackageShare("prop_arm_description"), "launch", "publish_urdf.launch.py"
    ])
    spawn_launch = PathJoinSubstitution([
        FindPackageShare("prop_arm_gazebo"), "launch", "spawn_prop_arm_description.launch.py"
    ])
    
    # Controller spawner:
    controller_launch = PathJoinSubstitution([
        FindPackageShare("prop_arm_gazebo_control"), "launch", "load_controller.launch.py"
    ])

    return LaunchDescription([

        # Launch Gazebo first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
        ),

        # Publish URDF (robot_description)
        TimerAction(
            period=2.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(urdf_launch)
            )]
        ),

        # Spawn the robot model in Gazebo
        TimerAction(
            period=4.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch)
            )]
        ),

        # Load the controller
        TimerAction(
            period=6.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller_launch)
            )]
        ),
    ])

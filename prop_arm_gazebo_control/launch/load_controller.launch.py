#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------- PYTHON IMPORTS ----------------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


# ---------------------- LAUNCH DESCRIPTION ----------------------
def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Bridges first so /clock and motor speed are available:
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Bridges:
    motor_speed_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='motor_speed_bridge',
        arguments=['/prop_arm/command/motor_speed@std_msgs/msg/Float64[gz.msgs.Double'],
        output='screen',
    )

    # Controller spawners:
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return [
        clock_bridge,
        motor_speed_bridge,
        TimerAction(period=2.0, actions=[jsb_spawner]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        OpaqueFunction(function=launch_setup)
    ])

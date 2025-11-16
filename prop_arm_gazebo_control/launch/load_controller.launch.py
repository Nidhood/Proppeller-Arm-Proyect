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

    # Package share dir and PID config_
    pkg_share = get_package_share_directory('prop_arm_gazebo_control')
    pid_cfg = os.path.join(pkg_share, 'config', 'prop_arm_pid_controller.yaml')

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

    # PID controller node:
    pid_controller_node = Node(
        package='prop_arm_gazebo_control',
        executable='pid_controller_node',
        name='prop_arm_pid_controller',
        output='screen',
        parameters=[pid_cfg, {'use_sim_time': use_sim_time}],
    )

    return [
        clock_bridge,
        motor_speed_bridge,
        TimerAction(period=2.0, actions=[jsb_spawner]),
        TimerAction(period=2.5, actions=[pid_controller_node]),
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

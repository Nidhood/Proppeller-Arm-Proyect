#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Allow use_sim_time:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,

        # 1. Spawn joint_state_broadcaster:
        TimerAction(
            period=2.0,  
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_joint_state_broadcaster',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )]
        ),
        
        # 2. Spawn position_controller:
        TimerAction(
            period=8.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_position_controller',
                arguments=[
                    'position_controller',
                    '--controller-manager', '/controller_manager'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )]
        ),

        # 3. Spawn motor_force_controller:
        TimerAction(
            period=4.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_motor_force_controller',
                arguments=[
                    'motor_force_controller',
                    '--controller-manager', '/controller_manager'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )]
        ),

        # 4. Spawn velocity_controller:
        TimerAction(
            period=6.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                name='spawner_velocity_controller',
                arguments=[
                    'velocity_controller',
                    '--controller-manager', '/controller_manager'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )]
        ),

        # 5. Start the propeller force controller node:
        TimerAction(
            period=10.0,
            actions=[Node(
                package='prop_arm_gazebo_control',
                executable='propeller_force_controller_node',
                name='propeller_force_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )]
        ),
    ])
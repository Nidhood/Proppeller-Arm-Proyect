#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import sys

def launch_setup(context, *args, **kwargs):
    # Use simulated clock
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_angle = LaunchConfiguration('target_angle')
    
    # Get the actual target angle value
    target_angle_value = float(target_angle.perform(context))
    
    return [
        # 1) Joint state broadcaster - starts immediately
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

        # 2) Velocity controller - starts after joint state broadcaster
        TimerAction(
            period=4.0,
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

        # 3) Angle Hold Controller - starts after controllers are ready
        TimerAction(
            period=6.0,
            actions=[Node(
                package='prop_arm_gazebo_control',
                executable='angle_hold_controller_node', 
                name='angle_hold_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'theta_ref_deg': target_angle_value,
                    'K': [15.0, 8.0, 2.0],  # Tuned state feedback gains
                    'u_min': -785.0,        # Motor velocity limits (rad/s)
                    'u_max': 785.0,
                    'rate_hz': 250.0,       # Control loop frequency
                    'joint_name': 'arm_link_joint',
                    'velocity_topic': '/velocity_controller/commands',
                    # Model parameters (can be tuned)
                    'Ja': 0.002,    # Arm inertia
                    'Jm': 0.001,    # Motor inertia  
                    'Kt': 0.02,     # Motor torque constant
                    'La': 0.001,    # Inductance
                    'Ke': 0.02,     # Back EMF constant
                    'Km': 1.0,      # Motor constant
                    'Rm': 2.0,      # Motor resistance
                    'Rs': 0.5,      # Series resistance
                    'Kf': 0.1,      # Friction coefficient
                }],
                # Pass target angle as command line argument too
                arguments=[str(target_angle_value)]
            )]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'target_angle',
            default_value='45.0',
            description='Target angle in degrees (MIT frame: 0=horizontal, +90=up, -90=down)'
        ),
        
        # Use OpaqueFunction to access launch configuration values
        OpaqueFunction(function=launch_setup)
    ])
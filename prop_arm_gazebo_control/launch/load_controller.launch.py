#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Bridges first so /clock and motor speed are available
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    motor_speed_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='motor_speed_bridge',
        arguments=['/prop_arm/command/motor_speed@std_msgs/msg/Float64[gz.msgs.Double'],
        output='screen',
    )

    # Your VEmfPublisher (computes V_emf and Delta V_emf)
    v_emf_pub = Node(
        package='prop_arm_gazebo_control',
        executable='v_emf_publisher',
        name='v_emf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ke': 0.02,                         # V/(rad/s)
            'v_emf_eq': 0.0,                    # equilibrium V_emf (choose later)
            'lpf_tau': 0.01,                    # seconds
            'use_lpf': True,
            'in_topic': '/prop_arm/motor_speed_est', 
            'out_v_emf_topic': '/prop_arm/v_emf',
            'out_delta_topic': '/prop_arm/delta_v_emf',
        }],
    )

    # Controller spawners (unchanged)
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_velocity_controller',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Start order: bridges → JSB → velocity → VEmfPublisher
    return [
        clock_bridge,
        motor_speed_bridge,
        TimerAction(period=2.0, actions=[jsb_spawner]),
        TimerAction(period=4.0, actions=[vel_spawner]),
        TimerAction(period=4.5, actions=[v_emf_pub]),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock'),
        OpaqueFunction(function=launch_setup)
    ])
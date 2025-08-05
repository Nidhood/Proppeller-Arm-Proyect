#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Argumento para use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Encuentra la carpeta share/prop_arm_gazebo_control
    pkg_share = FindPackageShare('prop_arm_gazebo_control')
    
    # Construye la ruta al YAML de controllers
    controllers_yaml = PathJoinSubstitution([
        pkg_share, 'config', 'ros2_controllers.yaml'
    ])

    return LaunchDescription([
        use_sim_time_arg,
        
        # 1) Levanta el nodo controller_manager (ros2_control_node)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[controllers_yaml, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/controller_manager', '/controller_manager')
            ],
        ),
        
        # 2) Spawn joint_state_broadcaster primero
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_joint_state_broadcaster',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager', '/controller_manager'
                    ],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                )
            ]
        ),
        
        # 3) Luego spawn el controlador de posición
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_arm_position_controller',
                    arguments=[
                        'arm_position_controller',
                        '--controller-manager', '/controller_manager'
                    ],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                )
            ]
        ),
    ])
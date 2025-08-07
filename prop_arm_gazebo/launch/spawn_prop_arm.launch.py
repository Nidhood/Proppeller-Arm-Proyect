#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------- PYTHON IMPORTS ----------------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# ---------------------- LAUNCH DESCRIPTION ----------------------
def generate_launch_description():
    
    # allow sim‐time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # find your packages
    pkg_gz   = FindPackageShare("prop_arm_gazebo")
    pkg_desc = FindPackageShare("prop_arm_description")
    pkg_ctrl = FindPackageShare("prop_arm_gazebo_control")

    # launch files
    gazebo_launch     = PathJoinSubstitution([pkg_gz,   "launch", "start_world.launch.py"])
    urdf_launch       = PathJoinSubstitution([pkg_desc, "launch", "publish_urdf.launch.py"])
    spawn_launch      = PathJoinSubstitution([pkg_gz,   "launch", "spawn_prop_arm_description.launch.py"])
    controller_launch = PathJoinSubstitution([pkg_ctrl, "launch", "load_controller.launch.py"])

    return LaunchDescription([
        use_sim_time_arg,

        # 1. Start Gazebo:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 2. Bridge /clock from Gazebo:
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ],
            parameters=[{ 'use_sim_time': use_sim_time }],
            output='screen',
        ),

        # 3. Publish URDF:
        TimerAction(
            period=1.0,
            actions=[ IncludeLaunchDescription(
                PythonLaunchDescriptionSource(urdf_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ) ]
        ),

        # 4. Spawn the robot in Gazebo:
        TimerAction(
            period=3.0,
            actions=[ IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ) ]
        ),

        # 5. Load controllers:
        TimerAction(
            period=6.0,
            actions=[ IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ) ]
        ),
    ])

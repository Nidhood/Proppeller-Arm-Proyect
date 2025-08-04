#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch_ros.actions import Node

# ---------------------- LAUNCH DESCRIPTION ----------------------
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_position_controller",
                "--controller-manager", "/controller_manager"
            ],
            output="screen",
        )
    ])

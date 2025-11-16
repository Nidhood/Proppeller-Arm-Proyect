from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('prop_arm_gazebo_control')
    cfg = os.path.join(pkg, 'config', 'step_test_input.yaml')

    return LaunchDescription([
        Node(
            package='prop_arm_gazebo_control',
            executable='step_test_input_node',
            name='step_test_input',
            output='screen',
            parameters=[cfg],
        )
    ])

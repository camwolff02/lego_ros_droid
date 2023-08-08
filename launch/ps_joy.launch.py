from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'lego_ros_droid'

    joy_params = os.path.join(
        get_package_share_directory(pkg_name), 'config', 'ps.config.yaml')

    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        parameters = [joy_params],
    )

    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable = 'teleop_node',
        name = 'teleop_node',  # normally teleop_twist_joy_node
        parameters = [joy_params],
        remappings = [('/cmd_vel', '/diff_cont/cmd_vel_unstamped')],
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])

import launch 
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tag_detection',
            executable='dummy_camera',
            name='camera'
        ),
        Node(
            package='tag_detection',
            executable='detect_tag',
            name='detect_tag'
        ),
        Node(
            package='tag_detection',
            executable='move_to_tag',
            name='move_to_tag'
        ),
        Node(
            package='tag_detection',
            executable='dummy_driver',
            name='drive'
        )
    ])
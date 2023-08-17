from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tag_detection',
            namespace='autonomy',
            executable='dummy_camera',
            name='camera'
        ),
        Node(
            package='tag_detection',
            namespace='autonomy',
            executable='detect_tag',
            name='detect_tag'
        ),
        Node(
            package='tag_detetion',
            namespace='autonomy',
            executable='move_to_tag',
            name='move_to_tag'
        ),
        Node(
            package='tag_detection',
            namespace='autonomy',
            executable='dummy_drive',
            name='drive'
        )
    ])
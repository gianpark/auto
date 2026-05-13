from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linetracer_sim',
            executable='video_publisher',
            name='video_publisher'
        ),
        Node(
            package='linetracer_sim',
            executable='linetracer_sim',
            name='linetracer_sim'
        ),
    ])

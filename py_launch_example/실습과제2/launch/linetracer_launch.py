from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 영상 기반 가짜 카메라 발행 노드
        Node(
            package='linetracer_sim',
            executable='video_publisher',
            name='video_publisher'
        ),
        # 라인트레이서 메인 노드 (영상 구독 + 모터 제어)
        Node(
            package='linetracer_sim',
            executable='linetracer_sim',
            name='linetracer_sim'
        ),
    ])
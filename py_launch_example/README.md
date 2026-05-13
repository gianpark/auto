##실습과제3



#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument   # 명령행 인자 선언
from launch.substitutions import LaunchConfiguration # 인자 값 참조
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare # 패키지 경로 탐색
import os

def generate_launch_description():

    # ① 명령행 인자 선언 (실행 시 변경 가능한 파라미터들)
    channel_type  = LaunchConfiguration('channel_type',  default='serial')
    # 통신 방식: serial(USB) / udp(네트워크) 선택

    serial_port   = LaunchConfiguration('serial_port',   default='/dev/ttyUSB0')
    # LIDAR가 연결된 시리얼 포트 (USB 연결 시 보통 /dev/ttyUSB0)

    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    # C1 전용 baudrate: 460800 (A1은 115200, A3는 256000으로 기종마다 다름)

    frame_id      = LaunchConfiguration('frame_id',      default='laser')
    # 좌표 프레임 ID: TF 트리에서 LIDAR 센서의 이름

    inverted      = LaunchConfiguration('inverted',      default='false')
    # LIDAR 상하 반전 여부 (뒤집어 장착 시 true)

    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    # 각도 보정 활성화: 스캔 데이터의 각도 정확도 향상

    scan_mode     = LaunchConfiguration('scan_mode',     default='Sensitivity')
    # C1 스캔 모드: Sensitivity(고감도) / Standard 선택 가능

    # ② DeclareLaunchArgument: 위에서 선언한 변수들을 launch 인자로 등록
    # → ros2 launch ... serial_port:=/dev/ttyUSB1 처럼 실행 시 덮어쓰기 가능
    return LaunchDescription([

        DeclareLaunchArgument('channel_type',
            default_value=channel_type,
            description='통신 채널 타입 (serial 또는 udp)'),

        DeclareLaunchArgument('serial_port',
            default_value=serial_port,
            description='LIDAR 시리얼 포트 경로'),

        DeclareLaunchArgument('serial_baudrate',
            default_value=serial_baudrate,
            description='시리얼 통신 속도 (C1은 460800)'),

        DeclareLaunchArgument('frame_id',
            default_value=frame_id,
            description='LIDAR TF 프레임 ID'),

        DeclareLaunchArgument('inverted',
            default_value=inverted,
            description='LIDAR 상하 반전 여부'),

        DeclareLaunchArgument('angle_compensate',
            default_value=angle_compensate,
            description='각도 보정 활성화 여부'),

        DeclareLaunchArgument('scan_mode',
            default_value=scan_mode,
            description='C1 스캔 모드'),

        # ③ sllidar_node 실행: LIDAR 드라이버 노드
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{               # ④ 파라미터: 노드에 전달되는 설정값
                'channel_type':     channel_type,
                'serial_port':      serial_port,
                'serial_baudrate':  serial_baudrate,
                'frame_id':         frame_id,
                'inverted':         inverted,
                'angle_compensate': angle_compensate,
                'scan_mode':        scan_mode,
            }],
            output='screen'             # 노드 출력을 터미널에 표시
        ),

        # ⑤ rviz2 실행: LIDAR 스캔 데이터 시각화 도구
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(   # rviz 설정파일 경로 지정
                FindPackageShare('sllidar_ros2'), 'rviz', 'sllidar_ros2.rviz')
            ],
            output='screen'
        ),
    ])

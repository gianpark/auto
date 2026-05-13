# 실습과제3

## 개요

---

## 전체 소스코드 및 라인별 설명

```python
#!/usr/bin/env python3
# Python3 인터프리터로 실행

from launch import LaunchDescription
# LaunchDescription: 실행할 노드/액션 목록을 담는 컨테이너

from launch.actions import DeclareLaunchArgument
# DeclareLaunchArgument: 실행 시 명령행으로 전달 가능한 인자를 선언

from launch.substitutions import LaunchConfiguration
# LaunchConfiguration: 선언된 인자의 값을 참조할 때 사용

from launch_ros.actions import Node
# Node: ROS2 노드를 실행하는 액션

from launch_ros.substitutions import FindPackageShare
# FindPackageShare: 설치된 패키지의 share 디렉터리 경로를 탐색

import os
# os.path.join으로 rviz 설정파일 경로 조합에 사용


def generate_launch_description():
    # ROS2 launch 시스템이 호출하는 필수 함수
    # 반환값(LaunchDescription)에 담긴 노드/액션들이 순서대로 실행됨

    # ── ① LaunchConfiguration으로 변수 선언 (기본값 포함) ──────────────

    channel_type = LaunchConfiguration('channel_type', default='serial')
    # 통신 채널 타입: serial(USB 시리얼) 또는 udp(네트워크) 선택
    # RPLIDAR C1은 USB 연결이므로 기본값 'serial'

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    # LIDAR가 연결된 시리얼 포트 경로
    # USB 연결 시 보통 /dev/ttyUSB0, 여러 장치 연결 시 ttyUSB1 등으로 변경

    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    # 시리얼 통신 속도 (bps)
    # C1 전용값: 460800 (A1=115200, A3=256000 으로 기종마다 다름)

    frame_id = LaunchConfiguration('frame_id', default='laser')
    # LIDAR 데이터의 TF 좌표 프레임 ID
    # rviz2, SLAM 등에서 이 이름으로 센서 위치를 참조함

    inverted = LaunchConfiguration('inverted', default='false')
    # LIDAR 상하 반전 여부
    # LIDAR를 뒤집어 장착했을 경우 'true'로 설정

    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    # 각도 보정 활성화 여부
    # 'true'로 설정 시 스캔 데이터의 각도 정확도가 향상됨

    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    # C1 스캔 모드 선택
    # Sensitivity: 고감도 모드 (기본값)
    # Standard: 표준 모드

    # ── ② LaunchDescription 반환 ───────────────────────────────────────
    return LaunchDescription([

        # ── ③ DeclareLaunchArgument: 명령행 인자 등록 ──────────────────
        # 아래 선언들 덕분에 실행 시 인자를 덮어쓸 수 있음
        # 예: ros2 launch sllidar_ros2 view_sllidar_c1_launch.py serial_port:=/dev/ttyUSB1

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='통신 채널 타입 (serial 또는 udp)'),
        # 'channel_type'이라는 이름으로 인자를 launch 시스템에 등록

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='LIDAR 시리얼 포트 경로'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='시리얼 통신 속도 (C1은 460800)'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='LIDAR TF 좌표 프레임 ID'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='LIDAR 상하 반전 여부'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='각도 보정 활성화 여부'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='C1 스캔 모드 (Sensitivity 또는 Standard)'),

        # ── ④ sllidar_node 실행: LIDAR 드라이버 노드 ───────────────────
        Node(
            package='sllidar_ros2',       # 패키지 이름
            executable='sllidar_node',    # 실행할 바이너리 이름
            name='sllidar_node',          # 노드 이름 (ros2 node list에 표시)
            parameters=[{
                # ③에서 선언한 인자값들을 노드 파라미터로 전달
                # 노드 내부에서 this->get_parameter()로 읽어 사용
                'channel_type':     channel_type,
                'serial_port':      serial_port,
                'serial_baudrate':  serial_baudrate,
                'frame_id':         frame_id,
                'inverted':         inverted,
                'angle_compensate': angle_compensate,
                'scan_mode':        scan_mode,
            }],
            output='screen'
            # 노드의 로그 출력을 터미널 화면에 표시
        ),

        # ── ⑤ rviz2 실행: LIDAR 스캔 데이터 시각화 ────────────────────
        Node(
            package='rviz2',              # rviz2 패키지
            executable='rviz2',           # rviz2 실행 파일
            name='rviz2',
            arguments=['-d', os.path.join(
                FindPackageShare('sllidar_ros2'),
                # sllidar_ros2 패키지의 share 디렉터리 경로 자동 탐색
                'rviz',
                'sllidar_ros2.rviz'
                # 미리 설정된 rviz 설정파일 로드 (LaserScan 토픽 등 설정 포함)
            )],
            output='screen'
        ),
    ])
```

---


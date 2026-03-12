cmake_minimum_required(VERSION 3.8)
project(line_publisher)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 필수 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 2. 헤더 파일 경로 포함 (Jazzy의 실제 경로를 직접 추가)
include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/jazzy/include/cv_bridge
  /opt/ros/jazzy/include/vision_opencv
)

# 3. 실행 파일 생성
add_executable(line_publisher src/line_publisher.cpp)

# 4. ROS2 의존성 연결
ament_target_dependencies(line_publisher
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# 5. 라이브러리 링크 (cv_bridge와 OpenCV를 확실히 연결)
target_link_libraries(line_publisher
  ${OpenCV_LIBRARIES}
  cv_bridge::cv_bridge
)

# 6. 설치 설정
install(TARGETS
  line_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

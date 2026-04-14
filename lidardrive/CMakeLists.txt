cmake_minimum_required(VERSION 3.8)
project(lidardrive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lidardrive src/lidardrive.cpp)
target_include_directories(lidardrive PUBLIC ${OpenCV_INCLUDE_DIRS})
target_compile_features(lidardrive PUBLIC cxx_std_17)
ament_target_dependencies(lidardrive rclcpp sensor_msgs geometry_msgs)
target_link_libraries(lidardrive ${OpenCV_LIBS})

install(TARGETS lidardrive
  DESTINATION lib/${PROJECT_NAME})

ament_package()
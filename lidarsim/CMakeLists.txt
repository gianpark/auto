cmake_minimum_required(VERSION 3.8)
project(lidarsim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lidarsim src/lidarsim.cpp)
target_include_directories(lidarsim PUBLIC ${OpenCV_INCLUDE_DIRS})
target_compile_features(lidarsim PUBLIC cxx_std_17)
ament_target_dependencies(lidarsim rclcpp sensor_msgs geometry_msgs)
target_link_libraries(lidarsim ${OpenCV_LIBS})

install(TARGETS lidarsim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
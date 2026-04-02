cmake_minimum_required(VERSION 3.8)
project(lidarplot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lidarplot src/lidarplot.cpp)

target_include_directories(lidarplot PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(lidarplot PUBLIC cxx_std_17)

ament_target_dependencies(lidarplot rclcpp sensor_msgs)

target_link_libraries(lidarplot ${OpenCV_LIBS})

install(TARGETS lidarplot DESTINATION lib/${PROJECT_NAME})

ament_package()
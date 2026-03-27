cmake_minimum_required(VERSION 3.8)
project(lanefollow_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# video_publisher
add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher
  rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(video_publisher ${OpenCV_LIBS})
target_compile_features(video_publisher PUBLIC cxx_std_17)

# lanefollow_sim
add_executable(lanefollow_sim src/lanefollow_sim.cpp)
ament_target_dependencies(lanefollow_sim
  rclcpp sensor_msgs std_msgs cv_bridge image_transport OpenCV)
target_link_libraries(lanefollow_sim ${OpenCV_LIBS})
target_compile_features(lanefollow_sim PUBLIC cxx_std_17)

install(TARGETS
  video_publisher
  lanefollow_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

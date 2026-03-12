cmake_minimum_required(VERSION 3.8)
project(line_subscriber)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(line_subscriber src/line_subscriber.cpp)
ament_target_dependencies(line_subscriber rclcpp sensor_msgs cv_bridge image_transport)

target_link_libraries(line_subscriber ${OpenCV_LIBS})
target_compile_features(line_subscriber PUBLIC cxx_std_14)


install(TARGETS
  
  line_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
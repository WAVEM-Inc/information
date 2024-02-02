cmake_minimum_required(VERSION 3.5)
project(battery_listener)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(battery_listener_node
  src/main.cpp
)
ament_target_dependencies(battery_listener_node rcl rclcpp sensor_msgs)


install(TARGETS
battery_listener_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

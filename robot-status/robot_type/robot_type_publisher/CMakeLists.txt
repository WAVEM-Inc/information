cmake_minimum_required(VERSION 3.5)
project(robot_type_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(robot_type_msgs)
include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(robot_type_publisher_node
src/main.cpp
src/MGR/robot_type_manager.cpp
)

ament_target_dependencies(robot_type_publisher_node rcl rclcpp robot_type_msgs)

install(TARGETS
  robot_type_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY 
  param
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

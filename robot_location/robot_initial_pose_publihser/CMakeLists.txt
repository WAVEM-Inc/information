cmake_minimum_required(VERSION 3.5)
project(robot_initial_pose_publihser)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(robot_initial_pose_publihser_node src/main.cpp src/InitPose.cpp)
ament_target_dependencies(robot_initial_pose_publihser_node rclcpp geometry_msgs)

install(TARGETS
  robot_initial_pose_publihser_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()

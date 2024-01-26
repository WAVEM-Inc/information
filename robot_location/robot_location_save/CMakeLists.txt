cmake_minimum_required(VERSION 3.5)
project(robot_location_save)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)

add_executable(robot_location_save_node src/main.cpp src/RobotLocationSaveMGR.cpp)
ament_target_dependencies(robot_location_save_node geometry_msgs rclcpp)

install(TARGETS
  robot_location_save_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()

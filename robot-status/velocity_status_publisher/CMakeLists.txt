cmake_minimum_required(VERSION 3.5)
project(velocity_status_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_status_msgs REQUIRED)

add_executable(velocity_status_publisher_node 
  src/main.cpp
  src/can/lib.cpp src/can/can_adaptor.cpp src/can/can_send.cpp src/can/data_relayer.cpp src/can/can_dump.cpp src/can/can_manager.cpp
)

ament_target_dependencies(velocity_status_publisher_node 
rclcpp 
std_msgs 
robot_status_msgs
)

install(TARGETS
  velocity_status_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

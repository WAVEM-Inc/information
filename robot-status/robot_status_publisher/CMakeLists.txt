cmake_minimum_required(VERSION 3.5)
project(robot_status_publisher)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  /usr/local/include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gts_navigation_msgs REQUIRED)
find_package(robot_status_msgs REQUIRED)

set(source_sets
  src/robot_pose_publisher/robot_pose_publisher.cxx
  src/navigation_status_publisher/navigation_status_publisher.cxx
  src/robot_status_publisher/robot_status_publisher.cxx
  src/robot_task_publisher/robot_task_publisher.cxx
  src/sensor_status_publisher/sensor_status_publisher.cxx
  src/main.cxx
)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_msgs
  tf2_ros
  gts_navigation_msgs
  robot_status_msgs
)

set(executable_name robot_status_publisher)

add_executable(${executable_name}
  ${source_sets}
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  ${source_sets}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

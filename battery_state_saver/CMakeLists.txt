cmake_minimum_required(VERSION 3.8)
project(battery_state_saver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(battery_state_saver src/battery_state_saver.cpp)
ament_target_dependencies(battery_state_saver rclcpp sensor_msgs)

install(TARGETS
        battery_state_saver
        DESTINATION lib/${PROJECT_NAME})

ament_package()
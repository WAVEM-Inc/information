#ifndef ROBOT_STATUS_PUBLISHER_UTILS__HXX
#define ROBOT_STATUS_PUBLISHER_UTILS__HXX

/**
 * @brief include default cpp header files
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <memory>
#include <cstring>
#include <string>

/**
 * @brief include header files for rclcpp
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rcutils/logging_macros.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <robot_status_msgs/msg/sensor_status.hpp>
#include <robot_status_msgs/msg/velocity_status.hpp>
#include <robot_status_msgs/msg/navigation_status.hpp>
#include <robot_status_msgs/msg/task_status.hpp>
#include <robot_status_msgs/srv/register_task.hpp>
#include <gts_navigation_msgs/msg/navigation_status_stamped.hpp>
#include <gts_navigation_msgs/msg/navigation_result_stamped.hpp>


/**
 * ------------------------------------------------------
 * ------------------ RCL AREA START --------------------
 * ------------------------------------------------------
 */


static constexpr const char *RCL_NODE_NAME = "robot_status_publisher";
static constexpr const int &RCL_DEFAULT_QOS = 10;
static constexpr const int &RCL_STOP_FLAG = 0;
static constexpr const char *RCL_DEFAULT_CONST_CHAR = "";
static constexpr const float &RCL_DEFAULT_FLOAT = 0.0f;
static constexpr const double &RCL_DEFAULT_DOUBLE = 0.0;
static constexpr const char *RCL_PUBLISHER_FLAG = "publisher";
static constexpr const char *RCL_SUBSCRIPTION_FLAG = "subscription";
static constexpr const char *RCL_SERVICE_CLIENT_FLAG = "service_client";
static constexpr const char *RCL_SERVICE_SERVER_FLAG = "service_server";
static constexpr const char *RCL_ACTION_CLIENT_FLAG = "action_client";
static constexpr const char *RCL_ACTION_SERVER_FLAG = "action_server";



/**
 * --------------------------------------------------.---
 * ------------------- RCL AREA END ---------------------
 * ------------------------------------------------------
 */


/**
 * @brief define macros area
 */
#define RCLCPP_LINE_INFO() \
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_ERROR() \
    RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

#define RCLCPP_LINE_WARN() \
    RCUTILS_LOG_WARN_NAMED(RCL_NODE_NAME, "LINE : [%d]", __LINE__)

/**
 * @brief using namespace area
 */
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#endif
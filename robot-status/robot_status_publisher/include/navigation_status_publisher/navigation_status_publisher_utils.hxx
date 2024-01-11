#ifndef NAVIGATION_STATUS_PUBLISHER_UTILS__HXX
#define NAVIGATION_STATUS_PUBLISHER_UTILS__HXX

#include "robot_status_publisher/robot_status_publisher_utils.hxx"

static constexpr const char *NAVIGATION_STATUS_PUBLISHER_NAME = "robot_task_publisher";
static constexpr const char *RCL_BATTERY_STATE_SUBSCRIPTION_TOPIC = "/battery/state";
static constexpr const char *RCL_VELOCITY_STATE_SUBSCRIPTION_TOPIC = "/velocity/state";
static constexpr const char *RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC = "/gts_navigation/status";
static constexpr const char *RCL_GTS_NAVIGATION_RESULT_SUBSCRIPTION_TOPIC = "/gts_navigation/result";
static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_STOPPED_CODE = 0;
static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_RESUMED_CODE = 1;
static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_STARTED_CODE = 2;
static constexpr const char *RCL_NAVIGATE_TO_POSE_GOAL_STARTED_FLAG = "STARTED";
static constexpr const char *RCL_NAVIGATE_TO_POSE_GOAL_STOPPED_FLAG = "STOPPED";
static constexpr const char *RCL_NAVIGATE_TO_POSE_GOAL_RESUMED_FLAG = "RESUMED";
static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED_CODE = 4;
static constexpr const char *RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED_FLAG = "SUCCEEDED";
static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_CANCELED_CODE = 5;
static constexpr const char *RCL_NAVIGATE_TO_POSE_GOAL_CANCELED_FLAG = "CANCELED";
static constexpr const int &RCL_NAVIGATE_TO_POSE_GOAL_ABORTED_CODE = 6;
static constexpr const char *RCL_NAVIGATE_TO_POSE_GOAL_ABORTED_FLAG = "ABORTED";
static constexpr const char *RCL_GTS_NAVIGATION_TASK_STATUS_PUBLISHER_TOPIC = "/gts_navigation/task_status";
static constexpr const int8_t &RCL_GTS_NAVIGATION_TASK_STATUS_STOPPED_CODE = 0;
static constexpr const int8_t &RCL_GTS_NAVIGATION_TASK_STATUS_RESUMED_CODE = 1;
static constexpr const int8_t &RCL_GTS_NAVIGATION_TASK_STATUS_STARTED_CODE = 2;
static constexpr const int8_t &RCL_GTS_NAVIGATION_TASK_STATUS_COMPLETED_CODE = 4;
static constexpr const char *RCL_GTS_NAVIGATION_SUCCEEDED_FLAG = "success";
static constexpr const char *RCL_GTS_NAVIGATION_FAILED_FLAG = "fail";
static constexpr const char *RCL_GTS_NAVIGATION_TASK_STATUS_STARTED_FLAG = "started";
static constexpr const char *RCL_GTS_NAVIGATION_TASK_STATUS_ENDED_FLAG = "ended";
static constexpr const char *RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG = "move";
static constexpr const char *RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG = "wait";
static constexpr const char *RCL_GTS_NAVIGATION_TASK_STATUS_STOP_FLAG = "stop";

#endif
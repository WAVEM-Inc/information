#ifndef ROBOT_TASK_PUBLISHER_UTILS__HXX
#define ROBOT_TASK_PUBLISHER_UTILS__HXX

#include "robot_status_publisher/robot_status_publisher_utils.hxx"

static constexpr const char *ROBOT_TASK_PUBLISHER_NAME = "robot_task_publisher";
static constexpr const char *RCL_GET_TASK_STATUS_PUBLISHER_NAME = "/robot_task/status";
static constexpr const char *RCL_REGISTER_TASK_SERVICE_SERVER_NAME = "/robot_task/register";
static constexpr const char *RCL_REGISTER_TASK_REGISTER_KEY = "qAqwmrwskdfliqnwkfnlasdkfnlas";
static constexpr const int8_t &RCL_REGISTER_TASK_REGISTER_KEY_EMPTY_CODE = -100;
static constexpr const char *RCL_REGISTER_TASK_REGISTER_KEY_EMPTY_MESSAGE = "register_key is empty";
static constexpr const int8_t &RCL_REGISTER_TASK_REGISTER_KEY_INCONSISTENCY_CODE = -101;
static constexpr const char *RCL_REGISTER_TASK_REGISTER_KEY_INCONSISTENCY_MESSAGE = "register_key is inconsistent";
static constexpr const int8_t &RCL_REGISTER_TASK_REGISTER_KEY_SUCCESS_CODE = 100;
static constexpr const char *RCL_REGISTER_TASK_REGISTER_KEY_SUCCESS_MESSAGE = "task has been registered";

#endif
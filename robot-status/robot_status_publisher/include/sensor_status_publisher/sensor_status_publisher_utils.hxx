#ifndef SENSOR_STATUS_PUBLISHER_UTILS__HXX
#define SENSOR_STATUS_PUBLISHER_UTILS__HXX

#include "robot_status_publisher/robot_status_publisher_utils.hxx"

static constexpr const char *RCL_SENSOR_STATUS_PUBLISHER_NAME = "sensor_status_publisher";
static constexpr const char *RCL_IMU_STATUS_HEADER_FRAME_ID = "imu_status";
static constexpr const int32_t &RCL_IMU_STATUS_NULL_STATUS_CODE = -1000;
static constexpr const char *RCL_IMU_STATUS_NULL_STATUS_MESSAGE = "IMU is NULL";
static constexpr const int32_t &RCL_IMU_STATUS_OK_STATUS_CODE = 1000;
static constexpr const char *RCL_IMU_STATUS_OK_STATUS_MESSAGE = "IMU is OK";
static constexpr const char *RCL_IMU_STATUS_PUBLISHER_TOPIC = "/imu/data/status";
static constexpr const char *RCL_IMU_SUBSCRIPTION_TOPIC = "/imu/data";
static constexpr const char *RCL_SCAN_STATUS_HEADER_FRAME_ID = "scan_status";
static constexpr const int32_t &RCL_SCAN_STATUS_NULL_STATUS_CODE = -1001;
static constexpr const char *RCL_SCAN_STATUS_NULL_STATUS_MESSAGE = "LiDAR is NULL";
static constexpr const int32_t &RCL_SCAN_STATUS_OK_STATUS_CODE = 1001;
static constexpr const char *RCL_SCAN_STATUS_OK_STATUS_MESSAGE = "LiDAR is OK";
static constexpr const char *RCL_SCAN_STATUS_PUBLISHER_TOPIC = "/scan/multi/status";
static constexpr const char *RCL_SCAN_SUBSCRIPTION_TOPIC = "/scan/multi";
static constexpr const char *RCL_GPS_STATUS_HEADER_FRAME_ID = "gps_status";
static constexpr const int32_t &RCL_GPS_STATUS_NULL_STATUS_CODE = -1002;
static constexpr const char *RCL_GPS_STATUS_NULL_STATUS_MESSAGE = "GPS is NULL";
static constexpr const int32_t &RCL_GPS_STATUS_OK_STATUS_CODE = 1002;
static constexpr const char *RCL_GPS_STATUS_OK_STATUS_MESSAGE = "GPS is OK";
static constexpr const char *RCL_GPS_STATUS_PUBLISHER_TOPIC = "/ublox/fix/status";
static constexpr const char *RCL_GPS_SUBSCRIPTION_TOPIC = "/ublox/fix";
static constexpr const char *RCL_BATTERY_STATE_STATUS_HEADER_FRAME_ID = "battery_state_status";
static constexpr const int32_t &RCL_BATTERY_ERROR_STATUS_STATUS_CODE = -1003;
static constexpr const char *RCL_BATTERY_STATE_ERROR_STATUS_MESSAGE = "Battery is Broken";
static constexpr const int32_t &RCL_BATTERY_STATE_STATUS_OK_STATUS_CODE = 1003;
static constexpr const char *RCL_BATTERY_STATE_STATUS_OK_STATUS_MESSAGE = "Battery is OK";
static constexpr const char *RCL_BATTERY_STATE_STATUS_PUBLISHER_TOPIC = "/battery/state/status";
static constexpr const char *RCL_BATTERY_ERROR_STATUS_SUBSCRIPTION_TOPIC = "/battery/err/status";
static constexpr const int32_t &RCL_BATTERY_ALRAM_ERROR_STATUS_CODE = -1004;
static constexpr const int32_t &RCL_BATTERY_MOTOR_POWER_LIMIT_STATUS_CODE = -1005;
static constexpr const int32_t &RCL_BATTERY_PACK_ERROR_STATUS_CODE = -1006;

#endif

#ifndef CAN_DEFINE_H
#define CAN_DEFINE_H

#define  CNV_SPEED_FACTOR 3.6
#define  RESOLUTION_SPEED_CTRL 10
#define  RESOLUTION_STEERING_CTRL 10.0

#define  OFFSET_STEERING 30
#define  OFFSET_STRANGLE 3000
#define  MAX_STEERING 30
#define  MIN_STEERING -30

enum MSG_ID {
  VCU_VEHICLE_STATUS_2 = 0x304,
  VCU_METER_REQ = 0x712
};

enum SVC_ID {
  CONTROL_STEERING,
  CONTROL_ACCELERATE,
  CONTROL_HARDWARE,
  RPM_FEEDBACK
};

enum ACC_GEAR {
   PARKING,
   FORWARD,
   NEUTRAL,
   REVERSE
};

enum CHANNEL_TYPE { CAN0,CAN1 } ;

/**
 * @brief Test device type
 * 
static const char *device_type[] =
        { "can0", "can1"};
*/
/**
 * @brief Test device type
 * 
 */
static const char *device_type[] =
        { "can0", "can1"};
#define CAN_ALIVE_CHECKTIME 2 // second
#define CAN_RECV_RETRY_TIME 2 // second
#define VELOCITY_STANDARD 800
#endif

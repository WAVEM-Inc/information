
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
  VCU_EPS_CONTROL_REQUEST = 0x169,
  REMOTE_CONTROL_SHAKE_2 = 0x10b,
  REMOTE_CONTROL_IO = 0x10a,
  DBS_STATUS = 0x142,
  VCU_DBS_REQUEST = 0x154,
  IECU_CONTROL_HARDWARE	=	0X506,
  IECU_CONTROL_ACCELERATE	=	0X504,
  IECU_CONTROL_BRAKE	=	0X503,
  IECU_CONTROL_STEERING	=	0X502,
  MODE_CONTROL_FLAG	=	0X501,
  TORQUE_FEEDBACK = 0x10,
  SYSTEM_INFO = 0xa0
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

#endif

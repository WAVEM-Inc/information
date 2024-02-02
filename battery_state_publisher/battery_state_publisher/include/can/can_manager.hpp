#ifndef CAN_MANAGER
#define CAN_MANAGER

// ros 
#include"rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
//can
#include"can/df_topic.hpp"
// can header file
#include "can/data_relayer.hpp"
#include "can/i_can_connect.hpp"

// linux header file 
#include <unistd.h> //sleep

#include <iomanip>
#include <chrono> // time
//extern int optind, opterr, optopt;
//static char *progname;
static volatile int state = 1;
/**
 * @brief 
 * @param signo 
 */
static void sigterm(int signo)
{
  fprintf(stdout, "SIGNAL %d  in main\n", signo);
	state = 0;
}
/**
 * @brief Class to control CAN communication
 * @author changunAn(changun516@wavem.net)
 * @see enable_shared_from_this
 * @see ICanConnect
 * @see IMotionColleague
 * @warning Be careful of problems caused by Mutex
 */
class CanMGR : public ICanConnect, public std::enable_shared_from_this<CanMGR>, public rclcpp::Node{
    private :
        DataRelayer data_relayer_; // Member variables for calling Can-related functions
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
        int fn_can_init(); // can callback function register 
        void battery_callback(int voltage,int max_temperature,int status);

        const std::string tp_battery_;
        const std::string tp_frame_id_;
        void log(std::string call_name);
    public :
        CanMGR();
        virtual ~CanMGR() noexcept;
        void fn_can_run();
 

};

#endif
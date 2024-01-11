#ifndef CAN_MANAGER
#define CAN_MANAGER

// ros 
#include"rclcpp/rclcpp.hpp" 
#include"robot_status_msgs/msg/velocity_status.hpp"
//can
#include"can/df_topic.hpp"
// can header file
#include "can/data_relayer.hpp"
#include "can/i_can_connect.hpp"

// linux header file 
#include <unistd.h> //sleep
#include <iomanip>
#include <chrono> // time

// enetiy header file
#include "entity/uvc.hpp"

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
        rclcpp::Publisher<robot_status_msgs::msg::VelocityStatus>::SharedPtr pub_velocity_;
        std::unique_ptr<ENTITY::UVC> uvc_;
        int fn_can_init(); // can callback function register 
        void velocity_callback(double speed);

        const std::string tp_velocity_;
        const std::string tp_frame_id_;
        void log(std::string call_name);
    public :
        CanMGR();
        virtual ~CanMGR() noexcept;
        void fn_can_run();
 

};

#endif
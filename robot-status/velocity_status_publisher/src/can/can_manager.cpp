#include"can/can_manager.hpp"

/**
 * @brief Construct a new Can M G R:: Can M G R object
 * @author changunAn(changun516@wavem.net)
 * @param motion_mediator 
 */
CanMGR::CanMGR()
: Node("velocity_status_publisher_node")
, tp_velocity_(TP_VELOCITY)
, tp_frame_id_(TP_FRAME_ID) {
    pub_velocity_ = this->create_publisher<robot_status_msgs::msg::VelocityStatus>(tp_velocity_,1);
    uvc_ = std::make_unique<ENTITY::UVC>();
    fn_can_init();
  
}

CanMGR::~CanMGR(){
    
}

/**
 * @brief can operation function
 * @author changunAn(changun516@wavem.net)
 */
void CanMGR::fn_can_run() {
	std::cout << "***can run start!!!***" << '\n';
	while(state){
		sleep(5);
	}
	std::cout << "***can end!!!***" << '\n';
}

/**
 * @brief Register callback function for can communication response
 * @return int Verify function normal termination
 * @author changunAn(changun516@wavem.net)
 * @date 23.04.06
 */
int CanMGR::fn_can_init() {
    data_relayer_.RegistVelocityCallback<CanMGR>(this, &CanMGR::velocity_callback);
    data_relayer_.Run();
    return 0;
}

/**
 * @brief can battery_callback
 * @author reidlo
*/
void CanMGR::velocity_callback(double speed) {
    robot_status_msgs::msg::VelocityStatus temp_velocity;
    double prev_velocity = uvc_->get_prev_velocity();
    rclcpp::Time now = this->now();
    if(prev_velocity==0 && speed==0){
        temp_velocity.distance=0;
    }
    else{
        rclcpp::Duration time_diff = now- uvc_->get_prev_time();
        double diff_time = time_diff.seconds();
        double diff_speed = speed - prev_velocity;
        uvc_->add_distance(std::fabs(diff_speed*diff_time));
    }

    std::cout << "[main] callback battery_callback : " << (double)speed<< '\n'; 

    temp_velocity.current_velocity=speed/3.6;
    temp_velocity.header.stamp = now;
    temp_velocity.distance = uvc_->get_acc_distance(); 
    temp_velocity.header.frame_id = tp_frame_id_;
    pub_velocity_->publish(temp_velocity);

    uvc_->set_perv_velocity(speed);
    uvc_->set_prev_time(this->now());
}




void CanMGR::log(std::string call_name) {
    std::chrono::system_clock::time_point callback_time = std::chrono::system_clock::now();
    std::time_t callback_time_t = std::chrono::system_clock::to_time_t(callback_time);
    std::tm* callback_time_tm = std::localtime(&callback_time_t);
    auto duration = callback_time.time_since_epoch();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration) % std::chrono::seconds(1);

    std::cout <<'['<< call_name<<']' <<" Date and Time: ";
    std::cout << callback_time_tm->tm_year + 1900 << "-" << std::setw(2) << std::setfill('0') << callback_time_tm->tm_mon + 1 << "-" << std::setw(2) << std::setfill('0') << callback_time_tm->tm_mday;
    std::cout << " " << std::setw(2) << std::setfill('0') << callback_time_tm->tm_hour << ":" << std::setw(2) << std::setfill('0') << callback_time_tm->tm_min << ":" << std::setw(2) << std::setfill('0') << callback_time_tm->tm_sec;
    std::cout << "." << std::setw(9) << std::setfill('0') << nanoseconds.count();
    std::cout << '\n';
}

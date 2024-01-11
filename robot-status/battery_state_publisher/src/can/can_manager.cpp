#include"can/can_manager.hpp"
#include"robot_status_msgs/msg/sensor_status.hpp"
/**
 * @brief Construct a new Can M G R:: Can M G R object
 * @author changunAn(changun516@wavem.net)
 * @param motion_mediator 
 */
CanMGR::CanMGR()
: Node("BatteryStatePublish")
, tp_battery_(TP_BATTERY)
, tp_frame_id_(TP_FRAME_ID) {
    pub_battery_ = this->create_publisher<sensor_msgs::msg::BatteryState>(tp_battery_,1);
    pub_battery_err_ = this->create_publisher<robot_status_msgs::msg::SensorStatus>(TP_BATTERY_ERR,1);
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
    data_relayer_.RegistBatteryCallback<CanMGR>(this, &CanMGR::battery_callback);
    data_relayer_.Run();
    return 0;
}

/**
 * @brief can battery_callback
 * @author reidlo
*/
void CanMGR::battery_callback(int voltage,int sys_status,int status) {
    //std::cout << "[main] callback battery_callback : " << (int)voltage<< '\n'; 
    sensor_msgs::msg::BatteryState battery;
    if(std::fabs(voltage)>0.0001){
        battery.percentage=static_cast<float>(voltage);
        battery.temperature = static_cast<float>(sys_status);
        battery.charge = static_cast<float>(status);
        battery.header.stamp = this->now();
        battery.header.frame_id = tp_frame_id_;
        pub_battery_->publish(battery);
    }
    if(battery.percentage<20){
        data_relayer_.ControlHardWare(true,false,false,false);
    }

    robot_status_msgs::msg::SensorStatus err_status;
    std::cout<<'!'<<sys_status<<std::endl;
    if(sys_status!=0){
        err_status.header.frame_id=TP_ERR_FRAME_ID;
        err_status.header.stamp= this->now();
        switch (sys_status)
        {
        case 1:
            err_status.status_code=-1004;
            err_status.status_message="Level 1 fault (alarm)";
            pub_battery_err_->publish(err_status);
            break;
        case 2:
            err_status.status_code=-1005;
            err_status.status_message="Level 2 Fault (motor power limit)";
            pub_battery_err_->publish(err_status);
            break;
        case 3:
            err_status.status_code=-1006;
            err_status.status_message="Level 3 Fault (battery pack stops output)";
            pub_battery_err_->publish(err_status);
            break;
        
        default:
            std::cout<<sys_status<<std::endl;
            break;
        }
    }
    
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

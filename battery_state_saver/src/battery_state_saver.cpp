#include <chrono>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "can_msgs/msg/vcu_vehicle_odometer_status.hpp"

using namespace std::chrono_literals;

class BatteryStateSaver : public rclcpp::Node
{
public:
    BatteryStateSaver()
            : Node("battery_state_saver"), dir_path_("/home/nuc-bt/RobotData/log/")
    {
        this->declare_parameter<std::string>("dir_path", dir_path_);
        this->get_parameter("dir_path", dir_path_);

        // Ensure the directory exists
        if (!directory_exists(dir_path_)) {
            if (!create_directory(dir_path_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", dir_path_.c_str());
                return;
            }
        }

        // Create callback groups
        callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create options for subscriber and timer
        auto subscriber_options = rclcpp::SubscriptionOptions();
        subscriber_options.callback_group = callback_group_subscriber_;
        auto subscriber_vcu_odom_options = rclcpp::SubscriptionOptions();
        subscriber_vcu_odom_options.callback_group = callback_group_odom_subscriber_;

        battery_state_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "/sensor/battery/state", 10, std::bind(&BatteryStateSaver::battery_state_callback, this, std::placeholders::_1), subscriber_options);

        vcu_odom_sub_ = this->create_subscription<can_msgs::msg::VcuVehicleOdometerStatus>("/drive/can/vcu/vehicle_odometer_status",
                                                                                           rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
                                                                                           std::bind(&BatteryStateSaver::vcu_odom_callback, this, std::placeholders::_1),
                                                                                           subscriber_vcu_odom_options);

        timer_ = this->create_wall_timer(
                5min, std::bind(&BatteryStateSaver::save_battery_state_to_file, this));

        // Initialize the file path with the current time
        file_path_ = dir_path_ + "battery_state_" + get_current_time_str() + ".txt";
    }

private:
    bool directory_exists(const std::string &path)
    {
        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            return false;
        } else if (info.st_mode & S_IFDIR) {
            return true;
        } else {
            return false;
        }
    }

    bool create_directory(const std::string &path)
    {
        if (mkdir(path.c_str(), 0777) == 0) {
            return true;
        } else {
            return false;
        }
    }

    std::string get_current_time_str()
    {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_c);

        std::stringstream ss;
        ss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
        return ss.str();
    }

    void battery_state_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Log Load");
        last_battery_state_ = msg;
    }
    void vcu_odom_callback(const can_msgs::msg::VcuVehicleOdometerStatus::SharedPtr msg){
        last_vcu_odom_ = msg;
    }
    void save_battery_state_to_file()
    {
        RCLCPP_INFO(this->get_logger(), "Log Save");
        if (last_battery_state_)
        {
            std::ofstream file(file_path_, std::ios_base::out | std::ios_base::app);
            if (file.is_open())
            {
                file << "Time: " << get_current_time_str() << "\n"
                     << "Percentage: " << last_battery_state_->voltage << "\n"
                     << "Voltage: " << last_battery_state_->percentage << "\n"
                     << "Odom: " <<last_vcu_odom_->odom_data<<"\n\n";
                file.close();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
    rclcpp::Subscription<can_msgs::msg::VcuVehicleOdometerStatus>::SharedPtr vcu_odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::BatteryState::SharedPtr last_battery_state_;
    can_msgs::msg::VcuVehicleOdometerStatus::SharedPtr last_vcu_odom_;
    std::string dir_path_;
    std::string file_path_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr callback_group_odom_subscriber_;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BatteryStateSaver>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

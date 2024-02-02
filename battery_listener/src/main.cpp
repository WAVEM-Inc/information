#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <iostream>
class BatterySubscriberNode : public rclcpp::Node {
public:
  BatterySubscriberNode() : Node("battery_subscriber") {
    subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery/state",
      10,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        // Battery state message callback
        RCLCPP_INFO(this->get_logger(), "Voltage: %.2f", msg->percentage);
      }
    );
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatterySubscriberNode>());
  rclcpp::shutdown();
  return 0;
}

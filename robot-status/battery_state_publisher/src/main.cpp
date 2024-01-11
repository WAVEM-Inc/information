#include "can/can_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanMGR>());
    rclcpp::shutdown();
    
    return 0;
}
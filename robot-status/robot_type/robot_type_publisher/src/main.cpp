#include"MGR/robot_type_manager.hpp"
int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotType::Manager>());
    rclcpp::shutdown();
    return 0;
}
#include"InitPose.hpp"
int main(int argc,char** argv){
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<InitPose>());
	rclcpp::shutdown();

    return 0;
}
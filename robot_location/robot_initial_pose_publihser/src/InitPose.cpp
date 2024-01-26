#include"InitPose.hpp"
#define FILE_PATH "/home/msi-341/RobotData/location.config"
InitPose::InitPose():Node("robot_initial_pose_publihser_node"){
    file_read_ = std::make_unique<FileRead>(FILE_PATH);
    publisher_ = this->create_publisher<InitMSG>("/initialpose", 10);
    run();
    rclcpp::shutdown();
}

void InitPose::run(){
    InitMSG init;
    std::vector<std::string> result =file_read_->read_file();
    if(result.size()==4){
        init.header.frame_id="/map";
        init.header.stamp = this->now();
        init.pose.pose.position.x = std::stod(result[0]);
        init.pose.pose.position.y = std::stod(result[1]);
        init.pose.pose.orientation.w = std::stod(result[2]);
        init.pose.pose.orientation.z= std::stod(result[3]);
        publisher_->publish(init);
    }

}



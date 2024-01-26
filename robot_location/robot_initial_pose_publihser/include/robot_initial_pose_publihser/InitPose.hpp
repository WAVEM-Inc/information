#ifndef ROBOT_LOCATION__ROBOT_INITIAL_POSE_PUBLISHER__INITPOSE__HPP_
#define ROBOT_LOCATION__ROBOT_INITIAL_POSE_PUBLISHER__INITPOSE__HPP_
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "FileRead.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include<memory.h>
using InitMSG = geometry_msgs::msg::PoseWithCovarianceStamped;
class InitPose : public rclcpp::Node{
    public :
        InitPose();
    private :
        std::unique_ptr<FileRead> file_read_;
        rclcpp::Publisher<InitMSG>::SharedPtr publisher_;
        void run();

};

#endif
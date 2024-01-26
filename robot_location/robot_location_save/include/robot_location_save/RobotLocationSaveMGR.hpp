#ifndef ROBOT_LOCATION_SAVE__RobotLocationSaveMGR__HPP_
#define ROBOT_LOCATION_SAVE__RobotLocationSaveMGR__HPP_
#include <iostream>
#include <memory.h>

#include "rclcpp/rclcpp.hpp"
#include "FileSave.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "Contant.hpp"
using PoseMSG = geometry_msgs::msg::Pose;
class RobotLocationSaveMGR : public rclcpp::Node{
    public :
        RobotLocationSaveMGR();
    private :  
        std::unique_ptr<FileSave> file_save_;
        std::unique_ptr<CONTANTS::Topic> contant_;
        rclcpp::Subscription<PoseMSG>::SharedPtr robot_pose_;
        void robot_pose_callback(std::shared_ptr<PoseMSG> pose);
        std::string posestamped_to_string(PoseMSG pose);
        
};


#endif

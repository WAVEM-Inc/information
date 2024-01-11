#ifndef ROBOT_TYPE_PUBLISHER__INCLUDE__MGR__ROBOT_TYPE_MANAGER__HPP_
#define ROBOT_TYPE_PUBLISHER__INCLUDE__MGR__ROBOT_TYPE_MANAGER__HPP_

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "robot_type_msgs/srv/type.hpp"
#include "Contants/Contants.hpp"
using RobotTypeSRV = robot_type_msgs::srv::Type;
namespace RobotType{
    class Manager : public rclcpp::Node{
        public :
            Manager();
        private :
            Contants::Type robot_type_;
            rclcpp::Service<RobotTypeSRV>::SharedPtr type_service_;
            void get_type(const std::shared_ptr<RobotTypeSRV::Request> req, const std::shared_ptr<RobotTypeSRV::Response> response);
    };
}
#endif
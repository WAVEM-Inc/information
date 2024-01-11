#ifndef ROBOT_STATUS_PUBLISHER__HXX
#define ROBOT_STATUS_PUBLISHER__HXX

#include "navigation_status_publisher/navigation_status_publisher.hxx"
#include "robot_pose_publisher/robot_pose_publisher.hxx"
#include "sensor_status_publisher/sensor_status_publisher.hxx"
#include "robot_task_publisher/robot_task_publisher.hxx"
#include "robot_status_publisher/robot_status_publisher_utils.hxx"

namespace robot_status_publisher
{
    namespace total
    {
        class RobotStatusPublisher final : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<robot_status_publisher::navigation::NavigationStatusPublisher> navigation_status_publisher_;
            std::shared_ptr<robot_status_publisher::pose::RobotPosePublisher> robot_pose_publisher_;
            std::shared_ptr<robot_status_publisher::sensor::SensorStatusPublisher> sensor_status_publisher_;
            std::shared_ptr<robot_status_publisher::robot_task::RobotTaskPublisher> robot_task_publisher_;
        public:
            explicit RobotStatusPublisher();
            virtual ~RobotStatusPublisher();
            static void signal_handler(int signal_input);
        };
    }
}

#endif
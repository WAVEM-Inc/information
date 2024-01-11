#ifndef ROBOT_TASK_PUBLISHER__HXX
#define ROBOT_TASK_PUBLISHER__HXX

#include "robot_task_publisher/robot_task_publisher_utils.hxx"

namespace robot_status_publisher
{
    namespace robot_task
    {
        class RobotTask final
        {
        private:
            std::string job_group_;
            std::string job_kind_;
            std::string job_plan_id_;
            std::string job_group_id_;
            std::string job_order_id_;
        public:
            explicit RobotTask();
            virtual ~RobotTask();
            std::string get__job_group();
            void set__job_group(std::string job_group);
            std::string get__job_kind();
            void set__job_kind(std::string job_kind);
            std::string get__job_plan_id();
            void set__job_plan_id(std::string job_plan_id);
            std::string get__job_group_id();
            void set__job_group_id(std::string job_group_id);
            std::string get__job_order_id();
            void set__job_order_id(std::string job_order_id);
        };

        class RobotTaskPublisher final
        {
        private:
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<robot_status_publisher::robot_task::RobotTask> robot_task_;

            rclcpp::CallbackGroup::SharedPtr robot_task_status_publisher_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr robot_task_status_publisher_timer_;

            rclcpp::CallbackGroup::SharedPtr robot_task_status_publisher_cb_group_;
            rclcpp::Publisher<robot_status_msgs::msg::TaskStatus>::SharedPtr robot_task_status_publisher_;

            rclcpp::Service<robot_status_msgs::srv::RegisterTask>::SharedPtr register_robot_task_status_service_;
        public:
            explicit RobotTaskPublisher(rclcpp::Node::SharedPtr main_node);
            virtual ~RobotTaskPublisher();
            void robot_task_status_publisher_timer_cb();
            void register_robot_task_status_service_request_cb(
                const std::shared_ptr<robot_status_msgs::srv::RegisterTask::Request> request,
                const std::shared_ptr<robot_status_msgs::srv::RegisterTask::Response> response);
        };
    }

}

#endif
#ifndef ROBOT_POSE_PUBLISHER__HXX
#define ROBOT_POSE_PUBLISHER__HXX

#include "robot_pose_publisher/robot_pose_publisher_utils.hxx"

namespace robot_status_publisher
{
    namespace pose
    {
        class RobotPosePublisher
        {
        private:
            bool is_stamped_;
            std::string map_frame_id_;
            std::string base_frame_id_;

            rclcpp::Node::SharedPtr node_;

            rclcpp::TimerBase::SharedPtr timer_;

            rclcpp::CallbackGroup::SharedPtr pose_stamped_publisher_cb_group_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;

            rclcpp::CallbackGroup::SharedPtr pose_publisher_cb_group_;
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            void flag_rcl_connections(const char *connection_type, const char *connection_name);
            std_msgs::msg::Header build_header(const char *header_frame_id);
            geometry_msgs::msg::Pose build_pose(geometry_msgs::msg::Vector3 vector3, geometry_msgs::msg::Quaternion quaternion);

            void timer_cb();

        public:
            explicit RobotPosePublisher(rclcpp::Node::SharedPtr main_node);
            virtual ~RobotPosePublisher();
        };
    }
}

#endif
#include "robot_status_publisher/robot_status_publisher.hxx"

int main(int argc, const char *const *argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<robot_status_publisher::total::RobotStatusPublisher>();
    rclcpp::executors::MultiThreadedExecutor multi_threaded_executor;
    multi_threaded_executor.add_node(node);
    multi_threaded_executor.spin();
    rclcpp::shutdown();

    return RCL_STOP_FLAG;
}
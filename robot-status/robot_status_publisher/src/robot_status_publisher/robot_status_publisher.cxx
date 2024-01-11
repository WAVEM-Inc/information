#include "robot_status_publisher/robot_status_publisher.hxx"

robot_status_publisher::total::RobotStatusPublisher::RobotStatusPublisher()
    : Node(RCL_NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] node has been created", RCL_NODE_NAME);
        RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s node", RCL_NODE_NAME);
        RCLCPP_LINE_ERROR();
        exit(RCL_STOP_FLAG);
    }

    this->navigation_status_publisher_ = std::make_shared<robot_status_publisher::navigation::NavigationStatusPublisher>(this->node_);
    this->robot_pose_publisher_ = std::make_shared<robot_status_publisher::pose::RobotPosePublisher>(this->node_);
    this->sensor_status_publisher_ = std::make_shared<robot_status_publisher::sensor::SensorStatusPublisher>(this->node_);
    this->robot_task_publisher_ = std::make_shared<robot_status_publisher::robot_task::RobotTaskPublisher>(this->node_);
}

robot_status_publisher::total::RobotStatusPublisher::~RobotStatusPublisher()
{
}

void robot_status_publisher::total::RobotStatusPublisher::signal_handler(int signal_input)
{
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "===== %s has been terminated with SIG [%d] =====", RCL_NODE_NAME, signal_input);
    signal(signal_input, SIG_IGN);
    exit(RCL_STOP_FLAG);
}
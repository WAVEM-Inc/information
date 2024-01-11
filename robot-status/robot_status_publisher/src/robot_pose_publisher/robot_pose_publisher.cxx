#include "robot_pose_publisher/robot_pose_publisher.hxx"

robot_status_publisher::pose::RobotPosePublisher::RobotPosePublisher(rclcpp::Node::SharedPtr main_node)
    : is_stamped_(false),
      map_frame_id_(RCL_MAP_FRAME_ID),
      base_frame_id_(RCL_BASE_LINK_FRAME_ID)
{
    this->node_ = main_node;

    rclcpp::Clock::SharedPtr clock = this->node_->get_clock();
    this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::Time time_now = this->node_->now();
    tf2::TimePoint tf2_time_point = tf2_ros::fromMsg(time_now);

    this->tf_buffer_->canTransform(
        RCL_MAP_FRAME_ID, RCL_BASE_LINK_FRAME_ID,
        tf2_time_point, std::chrono::seconds(1));

    this->node_->declare_parameter<std::string>("map_frame", "map");
    this->node_->declare_parameter<std::string>("base_frame", "base_link");
    this->node_->declare_parameter<bool>("is_stamped", false);
    this->node_->get_parameter("map_frame", this->map_frame_id_);
    this->node_->get_parameter("base_frame", this->base_frame_id_);
    this->node_->get_parameter("is_stamped", this->is_stamped_);

    if (this->is_stamped_)
    {
        this->pose_stamped_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::PublisherOptions pose_stamped_publisher_opts;
        pose_stamped_publisher_opts.callback_group = this->pose_stamped_publisher_cb_group_;
        this->pose_stamped_publisher_ = this->node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            RCL_ROBOT_POSE_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)),
            pose_stamped_publisher_opts);
    }
    else
    {
        this->pose_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::PublisherOptions pose_publisher_opts;
        pose_publisher_opts.callback_group = this->pose_publisher_cb_group_;
        this->pose_publisher_ = this->node_->create_publisher<geometry_msgs::msg::Pose>(
            RCL_ROBOT_POSE_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(1)),
            pose_publisher_opts);
    }

    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_ROBOT_POSE_TOPIC);

    this->timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&robot_status_publisher::pose::RobotPosePublisher::timer_cb, this));
}

robot_status_publisher::pose::RobotPosePublisher::~RobotPosePublisher()
{
}

void robot_status_publisher::pose::RobotPosePublisher::flag_rcl_connections(const char *connection_type, const char *connection_name)
{
    RCLCPP_INFO(this->node_->get_logger(), "RCL {%s} [%s - %s] created...", RCL_ROBOT_POSE_PUBLISHER_NAME, connection_type, connection_name);
    RCLCPP_LINE_INFO();
}

std_msgs::msg::Header robot_status_publisher::pose::RobotPosePublisher::build_header(const char *header_frame_id)
{
    std::chrono::_V2::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::_V2::system_clock::duration current_time_duration = current_time.time_since_epoch();

    const int32_t &current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time_duration).count();
    const int32_t &current_time_nanosec = current_time_sec % 1000000000;

    builtin_interfaces::msg::Time::UniquePtr stamp = std::make_unique<builtin_interfaces::msg::Time>();
    stamp->set__sec(current_time_sec);
    stamp->set__nanosec(current_time_nanosec);

    std_msgs::msg::Header::UniquePtr header = std::make_unique<std_msgs::msg::Header>();
    header->set__frame_id(header_frame_id);

    const builtin_interfaces::msg::Time &&stamp_moved = std::move(*stamp);
    header->set__stamp(stamp_moved);

    const std_msgs::msg::Header &&header_moved = std::move(*header);

    return header_moved;
}

geometry_msgs::msg::Pose robot_status_publisher::pose::RobotPosePublisher::build_pose(geometry_msgs::msg::Vector3 vector3, geometry_msgs::msg::Quaternion quaternion)
{
    geometry_msgs::msg::Pose::UniquePtr pose = std::make_unique<geometry_msgs::msg::Pose>();

    geometry_msgs::msg::Point::UniquePtr point = std::make_unique<geometry_msgs::msg::Point>();
    point->set__x(vector3.x);
    point->set__y(vector3.y);
    point->set__z(vector3.z);

    const geometry_msgs::msg::Point &&point_moved = std::move(*point);
    pose->set__position(point_moved);
    pose->set__orientation(quaternion);

    const geometry_msgs::msg::Pose &pose_moved = std::move(*pose);

    return pose_moved;
}

void robot_status_publisher::pose::RobotPosePublisher::timer_cb()
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    try
    {
        transform_stamped = this->tf_buffer_->lookupTransform(RCL_MAP_FRAME_ID, RCL_BASE_LINK_FRAME_ID, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &tf_expn)
    {
        // RCLCPP_INFO(this->node_->get_logger(), "timer_cb tf2_transform_exception : %s", tf_expn.what());
        return;
    }
    geometry_msgs::msg::PoseStamped::UniquePtr pose_stamped = std::make_unique<geometry_msgs::msg::PoseStamped>();

    std_msgs::msg::Header built_header = this->build_header(RCL_MAP_FRAME_ID);
    pose_stamped->set__header(built_header);

    geometry_msgs::msg::Pose built_pose = this->build_pose(transform_stamped.transform.translation, transform_stamped.transform.rotation);
    pose_stamped->set__pose(built_pose);

    if (is_stamped_)
    {
        const geometry_msgs::msg::PoseStamped &&pose_stamped_moved = std::move(*pose_stamped);
        this->pose_stamped_publisher_->publish(pose_stamped_moved);
    }
    else
    {
        this->pose_publisher_->publish(built_pose);
    }
}
#include "navigation_status_publisher/navigation_status_publisher.hxx"

robot_status_publisher::navigation::NavigationStatus::NavigationStatus()
    : start_time_(RCL_DEFAULT_CONST_CHAR),
      end_time_(RCL_DEFAULT_CONST_CHAR),
      start_battery_level_(RCL_DEFAULT_FLOAT),
      end_battery_level_(RCL_DEFAULT_FLOAT),
      start_dist_(RCL_DEFAULT_DOUBLE),
      end_dist_(RCL_DEFAULT_DOUBLE)
{
}

robot_status_publisher::navigation::NavigationStatus::~NavigationStatus()
{
}

int8_t robot_status_publisher::navigation::NavigationStatus::get__status_code()
{
    return this->status_code_;
}

void robot_status_publisher::navigation::NavigationStatus::set__status_code(int8_t status_code)
{
    this->status_code_ = status_code;
}

std::string robot_status_publisher::navigation::NavigationStatus::get__start_time()
{
    return this->start_time_;
}

void robot_status_publisher::navigation::NavigationStatus::set__start_time(std::string start_time)
{
    this->start_time_ = start_time;
}

std::string robot_status_publisher::navigation::NavigationStatus::get__end_time()
{
    return this->end_time_;
}

void robot_status_publisher::navigation::NavigationStatus::set__end_time(std::string end_time)
{
    this->end_time_ = end_time;
}

float robot_status_publisher::navigation::NavigationStatus::get__start_battery_level()
{
    return this->start_battery_level_;
}

void robot_status_publisher::navigation::NavigationStatus::set__start_battery_level(float start_battery_level)
{
    this->start_battery_level_ = start_battery_level;
}

float robot_status_publisher::navigation::NavigationStatus::get__end_battery_level()
{
    return this->end_battery_level_;
}

void robot_status_publisher::navigation::NavigationStatus::set__end_battery_level(float end_battery_level)
{
    this->end_battery_level_ = end_battery_level;
}

double robot_status_publisher::navigation::NavigationStatus::get__start_dist()
{
    return this->start_dist_;
}

void robot_status_publisher::navigation::NavigationStatus::set__start_dist(double start_dist)
{
    this->start_dist_ = start_dist;
}

double robot_status_publisher::navigation::NavigationStatus::get__end_dist()
{
    return this->end_dist_;
}

void robot_status_publisher::navigation::NavigationStatus::set__end_dist(double end_dist)
{
    this->end_dist_ = end_dist;
}

robot_status_publisher::navigation::NavigationStatusPublisher::NavigationStatusPublisher(rclcpp::Node::SharedPtr main_node)
{
    this->node_ = main_node;

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] sub_node has been created", NAVIGATION_STATUS_PUBLISHER_NAME);
        RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s sub_node", NAVIGATION_STATUS_PUBLISHER_NAME);
        RCLCPP_LINE_ERROR();
        exit(RCL_STOP_FLAG);
    }

    this->navigation_status_ = std::make_shared<robot_status_publisher::navigation::NavigationStatus>();

    this->battery_state_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions battery_state_subscription_opts;
    battery_state_subscription_opts.callback_group = this->battery_state_subscription_cb_group_;
    this->battery_state_subscription_ = this->node_->create_subscription<sensor_msgs::msg::BatteryState>(
        RCL_BATTERY_STATE_SUBSCRIPTION_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&robot_status_publisher::navigation::NavigationStatusPublisher::battery_state_subscription_cb, this, _1),
        battery_state_subscription_opts);
    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_BATTERY_STATE_SUBSCRIPTION_TOPIC);

    this->velocity_state_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions velocity_state_subscription_opts;
    velocity_state_subscription_opts.callback_group = this->velocity_state_subscription_cb_group_;
    this->velocity_state_subscription_ = this->node_->create_subscription<robot_status_msgs::msg::VelocityStatus>(
        RCL_VELOCITY_STATE_SUBSCRIPTION_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&robot_status_publisher::navigation::NavigationStatusPublisher::velocity_state_subscription_cb, this, _1),
        velocity_state_subscription_opts);
    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_VELOCITY_STATE_SUBSCRIPTION_TOPIC);

    this->gts_navigation_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gts_navigation_status_subscription_opts;
    gts_navigation_status_subscription_opts.callback_group = this->gts_navigation_status_subscription_cb_group_;
    this->gts_navigation_status_subscription_ = this->node_->create_subscription<gts_navigation_msgs::msg::NavigationStatusStamped>(
        RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&robot_status_publisher::navigation::NavigationStatusPublisher::gts_navigation_status_subscription_cb, this, _1),
        gts_navigation_status_subscription_opts);
    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC);

    this->gts_navigation_result_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gts_navigation_result_subscription_opts;
    gts_navigation_result_subscription_opts.callback_group = this->gts_navigation_result_subscription_cb_group_;
    this->gts_navigation_result_subscription_ = this->node_->create_subscription<gts_navigation_msgs::msg::NavigationResultStamped>(
        RCL_GTS_NAVIGATION_RESULT_SUBSCRIPTION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        std::bind(&robot_status_publisher::navigation::NavigationStatusPublisher::gts_navigation_result_subscription_cb, this, _1),
        gts_navigation_result_subscription_opts);
    this->flag_rcl_connections(RCL_SUBSCRIPTION_FLAG, RCL_GTS_NAVIGATION_RESULT_SUBSCRIPTION_TOPIC);

    this->navigation_task_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions navigation_task_publisher_opts;
    navigation_task_publisher_opts.callback_group = this->navigation_task_status_publisher_cb_group_;
    this->navigation_status_publisher_ = this->node_->create_publisher<robot_status_msgs::msg::NavigationStatus>(
        RCL_GTS_NAVIGATION_TASK_STATUS_PUBLISHER_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        navigation_task_publisher_opts);
    this->flag_rcl_connections(RCL_PUBLISHER_FLAG, RCL_GTS_NAVIGATION_TASK_STATUS_PUBLISHER_TOPIC);
}

robot_status_publisher::navigation::NavigationStatusPublisher::~NavigationStatusPublisher()
{
}

void robot_status_publisher::navigation::NavigationStatusPublisher::flag_rcl_connections(const char *connection_type, const char *connection_name)
{
    RCLCPP_INFO(this->node_->get_logger(), "RCL {%s} [%s - %s] created...", NAVIGATION_STATUS_PUBLISHER_NAME, connection_type, connection_name);
    RCLCPP_LINE_INFO();
}

void robot_status_publisher::navigation::NavigationStatusPublisher::battery_state_subscription_cb(sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb)
{
    this->battery_state_ = battery_state_cb;

    bool is_battery_state_nullptr = this->check_battery_state();

    if (is_battery_state_nullptr)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "{%s} subscription cb battery_state_ is nullptr", RCL_BATTERY_STATE_SUBSCRIPTION_TOPIC);
        RCLCPP_LINE_ERROR();
        return;
    }
}

bool robot_status_publisher::navigation::NavigationStatusPublisher::check_battery_state()
{
    bool is_battery_state_nullptr = (this->battery_state_ == nullptr);

    return is_battery_state_nullptr;
}

void robot_status_publisher::navigation::NavigationStatusPublisher::velocity_state_subscription_cb(robot_status_msgs::msg::VelocityStatus::SharedPtr velocity_state_cb)
{
    this->velocity_state_ = velocity_state_cb;

    bool is_velocity_state_nullptr = this->check_velocity_state();

    if (is_velocity_state_nullptr)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "{%s} subscription cb velocity_state_ is nullptr", RCL_VELOCITY_STATE_SUBSCRIPTION_TOPIC);
        RCLCPP_LINE_ERROR();
        return;
    }
}

bool robot_status_publisher::navigation::NavigationStatusPublisher::check_velocity_state()
{
    bool is_velocity_state_nullptr = (this->velocity_state_ == nullptr);

    return is_velocity_state_nullptr;
}

void robot_status_publisher::navigation::NavigationStatusPublisher::gts_navigation_status_subscription_cb(gts_navigation_msgs::msg::NavigationStatusStamped::SharedPtr gts_navigation_status_cb)
{
    RCLCPP_INFO(
        this->node_->get_logger(),
        "{%s} subscription cb status_code : [%d]",
        RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC,
        gts_navigation_status_cb->status_code);
    RCLCPP_LINE_INFO();

    bool is_battery_state_nullptr = this->check_battery_state();
    bool is_velocity_state_nullptr = this->check_velocity_state();

    if (is_battery_state_nullptr)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "{%s} subscription cb battery_state_ is nullptr", RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC);
        RCLCPP_LINE_ERROR();
        return;
    }

    if (is_velocity_state_nullptr)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "{%s} subscription cb velocity_state_ is nullptr", RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC);
        RCLCPP_LINE_ERROR();
        return;
    }

    const std::string &start_time = this->capture_current_time();
    const float &start_battery_percentage = this->battery_state_->percentage;
    const double &start_dist = this->velocity_state_->distance;
    const int &status_code = gts_navigation_status_cb->status_code;

    const char *goal_status_flag = "";

    if (status_code == RCL_NAVIGATE_TO_POSE_GOAL_STARTED_CODE)
    {
        this->navigation_status_->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_STARTED_CODE);
        goal_status_flag = RCL_NAVIGATE_TO_POSE_GOAL_STARTED_FLAG;
    }
    else if (status_code == RCL_NAVIGATE_TO_POSE_GOAL_STOPPED_CODE)
    {
        this->navigation_status_->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_STOPPED_CODE);
        goal_status_flag = RCL_NAVIGATE_TO_POSE_GOAL_STOPPED_FLAG;
        this->navigation_task_status_publish(RCL_NAVIGATE_TO_POSE_GOAL_STOPPED_FLAG, RCL_GTS_NAVIGATION_TASK_STATUS_STOP_FLAG, RCL_GTS_NAVIGATION_FAILED_FLAG);
    }
    else if (status_code == RCL_NAVIGATE_TO_POSE_GOAL_RESUMED_CODE)
    {
        this->navigation_status_->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_RESUMED_CODE);
        goal_status_flag = RCL_NAVIGATE_TO_POSE_GOAL_RESUMED_FLAG;
        this->navigation_task_status_publish(RCL_NAVIGATE_TO_POSE_GOAL_RESUMED_FLAG, RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG, RCL_GTS_NAVIGATION_SUCCEEDED_FLAG);
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "{%s} subscription cb, unknown status_code : [%d]... aborting", RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC, status_code);
        RCLCPP_LINE_ERROR();
    }

    this->navigation_status_->set__start_time(start_time);
    this->navigation_status_->set__start_battery_level(start_battery_percentage);
    this->navigation_status_->set__start_dist(start_dist);

    RCLCPP_INFO(
        this->node_->get_logger(),
        "{%s} subscription cb\n\tgoal_status : [%s]\n\tstart_battery_level : [%f]\n\tstart_time : [%s]\n\tstart_dist : [%f]",
        RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC,
        goal_status_flag,
        this->navigation_status_->get__start_battery_level(),
        this->navigation_status_->get__start_time().c_str(),
        this->navigation_status_->get__start_dist());
    RCLCPP_LINE_INFO();
}

void robot_status_publisher::navigation::NavigationStatusPublisher::gts_navigation_result_subscription_cb(gts_navigation_msgs::msg::NavigationResultStamped::SharedPtr gts_navigation_result_cb)
{
    const int &result_code = gts_navigation_result_cb->result_code;

    RCLCPP_INFO(this->node_->get_logger(), "{%s} subscription cb navigation_result : [%d]", RCL_GTS_NAVIGATION_RESULT_SUBSCRIPTION_TOPIC, result_code);
    RCLCPP_LINE_INFO();

    if (result_code == RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED_CODE)
    {
        this->navigation_status_->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_COMPLETED_CODE);
        this->navigation_task_status_publish(RCL_NAVIGATE_TO_POSE_GOAL_SUCCEEDED_FLAG, RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG, RCL_GTS_NAVIGATION_SUCCEEDED_FLAG);
        this->navigation_status_->set__start_time(RCL_DEFAULT_CONST_CHAR);
        this->navigation_status_->set__end_time(RCL_DEFAULT_CONST_CHAR);
        this->navigation_status_->set__start_battery_level(RCL_DEFAULT_FLOAT);
        this->navigation_status_->set__end_battery_level(RCL_DEFAULT_FLOAT);
        this->navigation_status_->set__end_dist(RCL_DEFAULT_DOUBLE);
    }
    else
    {
        RCLCPP_LINE_INFO();
        return;
    }
}

void robot_status_publisher::navigation::NavigationStatusPublisher::navigation_task_status_publish(const char *goal_status, const char *job, const char *status)
{
    bool is_battery_state_nullptr = this->check_battery_state();
    bool is_velocity_state_nullptr = this->check_velocity_state();

    if (is_battery_state_nullptr)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "{%s} subscription cb battery_state_ is nullptr", RCL_BATTERY_STATE_SUBSCRIPTION_TOPIC);
        RCLCPP_LINE_ERROR();
        return;
    }

    if (is_velocity_state_nullptr)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "{%s} subscription cb velocity_state_ is nullptr", RCL_VELOCITY_STATE_SUBSCRIPTION_TOPIC);
        RCLCPP_LINE_ERROR();
        return;
    }

    const std::string &current_time = this->capture_current_time();
    const float &end_battery_percentage = this->battery_state_->percentage;
    const double &end_dist = this->velocity_state_->distance;

    this->navigation_status_->set__end_time(current_time);
    this->navigation_status_->set__end_battery_level(end_battery_percentage);
    this->navigation_status_->set__end_dist(end_dist);

    RCLCPP_INFO(
        this->node_->get_logger(),
        "{%s} subscription cb\n\tgoal_status : [%s]\n\tstart_battery_level : [%f]\n\tend_battery_level : [%f]\n\tstart_time : [%s]\n\tend_time : [%s]\n\tstart_dist : [%f]\n\tend_dist : [%f]",
        RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC,
        goal_status,
        this->navigation_status_->get__start_battery_level(), this->navigation_status_->get__end_battery_level(),
        this->navigation_status_->get__start_time().c_str(), this->navigation_status_->get__end_time().c_str(),
        this->navigation_status_->get__start_dist(), this->navigation_status_->get__end_dist());
    RCLCPP_LINE_INFO();

    const int8_t &status_code = this->navigation_status_->get__status_code();
    bool is_navigation_started = (status_code == RCL_GTS_NAVIGATION_TASK_STATUS_STARTED_CODE);
    bool is_navigation_stopped = (status_code == RCL_GTS_NAVIGATION_TASK_STATUS_STOPPED_CODE);
    bool is_navigation_resumed = (status_code == RCL_GTS_NAVIGATION_TASK_STATUS_RESUMED_CODE);

    RCLCPP_INFO(
        this->node_->get_logger(),
        "{%s} subscription cb\n\tstatus_code : [%d]\n\tis_navigation_started : [%d]",
        RCL_GTS_NAVIGATION_STATUS_SUBSCRIPTION_TOPIC,
        status_code,
        is_navigation_started);
    RCLCPP_LINE_INFO();

    robot_status_msgs::msg::NavigationStatus::UniquePtr navigation_status = std::make_unique<robot_status_msgs::msg::NavigationStatus>();

    if (is_navigation_started)
    {
        navigation_status->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_STARTED_CODE);
        navigation_status->set__job_group(RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG);
        navigation_status->set__job_kind(RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG);
    }
    else if (is_navigation_stopped)
    {
        navigation_status->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_STOPPED_CODE);
        navigation_status->set__job_group(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG);
        navigation_status->set__job_kind(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG);
    }
    else if (is_navigation_resumed)
    {
        navigation_status->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_RESUMED_CODE);
        navigation_status->set__job_group(RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG);
        navigation_status->set__job_kind(RCL_GTS_NAVIGATION_TASK_STATUS_MOVE_FLAG);
    }
    else
    {
        navigation_status->set__status_code(RCL_GTS_NAVIGATION_TASK_STATUS_COMPLETED_CODE);
        navigation_status->set__job_group(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG);
        navigation_status->set__job_kind(RCL_GTS_NAVIGATION_TASK_STATUS_WAIT_FLAG);
    }

    navigation_status->set__status(status);
    navigation_status->set__start_time(this->navigation_status_->get__start_time());
    navigation_status->set__end_time(this->navigation_status_->get__end_time());
    navigation_status->set__start_battery_level(this->navigation_status_->get__start_battery_level());
    navigation_status->set__end_battery_level(this->navigation_status_->get__end_battery_level());
    navigation_status->set__start_dist(this->navigation_status_->get__start_dist());
    navigation_status->set__end_dist(this->navigation_status_->get__end_dist());

    const robot_status_msgs::msg::NavigationStatus &&navigation_status_moved = std::move(*navigation_status);

    this->navigation_status_publisher_->publish(navigation_status_moved);
}

std::string robot_status_publisher::navigation::NavigationStatusPublisher::capture_current_time()
{
    time_t raw_time;
    struct tm *time_info;

    time(&raw_time);
    time_info = localtime(&raw_time);

    char buffer[16];
    strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", time_info);

    std::string current_time = std::string(buffer);

    return current_time;
}
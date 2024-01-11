#include "robot_task_publisher/robot_task_publisher.hxx"

robot_status_publisher::robot_task::RobotTask::RobotTask()
    : job_group_(RCL_DEFAULT_CONST_CHAR),
      job_kind_(RCL_DEFAULT_CONST_CHAR),
      job_plan_id_(RCL_DEFAULT_CONST_CHAR),
      job_group_id_(RCL_DEFAULT_CONST_CHAR),
      job_order_id_(RCL_DEFAULT_CONST_CHAR)
{
}

robot_status_publisher::robot_task::RobotTask::~RobotTask()
{
}

std::string robot_status_publisher::robot_task::RobotTask::get__job_group()
{
    return this->job_group_;
}

void robot_status_publisher::robot_task::RobotTask::set__job_group(std::string job_group)
{
    this->job_group_ = job_group;
}

std::string robot_status_publisher::robot_task::RobotTask::get__job_kind()
{
    return this->job_kind_;
}

void robot_status_publisher::robot_task::RobotTask::set__job_kind(std::string job_kind)
{
    this->job_kind_ = job_kind;
}

std::string robot_status_publisher::robot_task::RobotTask::get__job_plan_id()
{
    return this->job_plan_id_;
}

void robot_status_publisher::robot_task::RobotTask::set__job_plan_id(std::string job_plan_id)
{
    this->job_plan_id_ = job_plan_id;
}

std::string robot_status_publisher::robot_task::RobotTask::get__job_group_id()
{
    return this->job_group_id_;
}

void robot_status_publisher::robot_task::RobotTask::set__job_group_id(std::string job_group_id)
{
    this->job_group_id_ = job_group_id;
}

std::string robot_status_publisher::robot_task::RobotTask::get__job_order_id()
{
    return this->job_order_id_;
}

void robot_status_publisher::robot_task::RobotTask::set__job_order_id(std::string job_order_id)
{
    this->job_order_id_ = job_order_id;
}

robot_status_publisher::robot_task::RobotTaskPublisher::RobotTaskPublisher(rclcpp::Node::SharedPtr main_node)
{
    this->node_ = main_node;

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] sub_node has been created", ROBOT_TASK_PUBLISHER_NAME);
        RCLCPP_LINE_INFO();
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s sub_node", ROBOT_TASK_PUBLISHER_NAME);
        RCLCPP_LINE_ERROR();
        exit(RCL_STOP_FLAG);
    }

    this->robot_task_ = std::make_shared<robot_status_publisher::robot_task::RobotTask>();

    this->robot_task_status_publisher_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->robot_task_status_publisher_timer_ = this->node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&robot_status_publisher::robot_task::RobotTaskPublisher::robot_task_status_publisher_timer_cb, this),
        this->robot_task_status_publisher_timer_cb_group_);

    this->robot_task_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions robot_task_status_publisher_opts;
    robot_task_status_publisher_opts.callback_group = this->robot_task_status_publisher_cb_group_;
    this->robot_task_status_publisher_ = this->node_->create_publisher<robot_status_msgs::msg::TaskStatus>(
        RCL_GET_TASK_STATUS_PUBLISHER_NAME,
        rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
        robot_task_status_publisher_opts
    );
    
    this->register_robot_task_status_service_ = this->node_->create_service<robot_status_msgs::srv::RegisterTask>(
        RCL_REGISTER_TASK_SERVICE_SERVER_NAME,
        std::bind(&robot_status_publisher::robot_task::RobotTaskPublisher::register_robot_task_status_service_request_cb, this, _1, _2));
}

robot_status_publisher::robot_task::RobotTaskPublisher::~RobotTaskPublisher()
{
}

void robot_status_publisher::robot_task::RobotTaskPublisher::robot_task_status_publisher_timer_cb()
{
    const std::string &job_group = this->robot_task_->get__job_group();
    const std::string &job_kind = this->robot_task_->get__job_kind();
    const std::string &job_plan_id = this->robot_task_->get__job_plan_id();
    const std::string &job_group_id = this->robot_task_->get__job_group_id();
    const std::string &job_order_id = this->robot_task_->get__job_order_id();

    // RCLCPP_INFO(
    //     this->node_->get_logger(),
    //     "{%s} publishing task_status every 1 seconds\n\tjob_group : [%s]\n\tjob_kind : [%s]\n\tjob_plan_id : [%s]\n\tjob_group_id : [%s]\n\tjob_order_id : [%s]",
    //     RCL_GET_TASK_STATUS_PUBLISHER_NAME,
    //     job_group.c_str(),
    //     job_kind.c_str(),
    //     job_plan_id.c_str(),
    //     job_group_id.c_str(),
    //     job_order_id.c_str());
    // RCLCPP_LINE_INFO();

    robot_status_msgs::msg::TaskStatus::UniquePtr task_status = std::make_unique<robot_status_msgs::msg::TaskStatus>();
    task_status->set__job_group(job_group);
    task_status->set__job_kind(job_kind);
    task_status->set__job_plan_id(job_plan_id);
    task_status->set__job_group_id(job_group_id);
    task_status->set__job_order_id(job_order_id);

    const robot_status_msgs::msg::TaskStatus &&task_status_moved = std::move(*task_status);
    this->robot_task_status_publisher_->publish(task_status_moved);
}

void robot_status_publisher::robot_task::RobotTaskPublisher::register_robot_task_status_service_request_cb(
    const std::shared_ptr<robot_status_msgs::srv::RegisterTask::Request> request,
    const std::shared_ptr<robot_status_msgs::srv::RegisterTask::Response> response)
{
    const std::string &request_register_key = request->register_key;

    bool is_request_register_key_empty = (request_register_key == "");
    bool is_request_register_key_valid = (request_register_key == RCL_REGISTER_TASK_REGISTER_KEY);

    if (is_request_register_key_empty)
    {
        response->set__result_code(RCL_REGISTER_TASK_REGISTER_KEY_EMPTY_CODE);
        response->set__result_message(RCL_REGISTER_TASK_REGISTER_KEY_EMPTY_MESSAGE);
    }
    else if (!is_request_register_key_valid)
    {
        response->set__result_code(RCL_REGISTER_TASK_REGISTER_KEY_INCONSISTENCY_CODE);
        response->set__result_message(RCL_REGISTER_TASK_REGISTER_KEY_INCONSISTENCY_MESSAGE);
    }
    else
    {
        const std::string &request_job_group = request->job_group;
        this->robot_task_->set__job_group(request_job_group);

        const std::string &request_job_kind = request->job_kind;
        this->robot_task_->set__job_kind(request_job_kind);

        const std::string &request_job_plan_id = request->job_plan_id;
        this->robot_task_->set__job_plan_id(request_job_plan_id);

        const std::string &request_job_group_id = request->job_group_id;
        this->robot_task_->set__job_group_id(request_job_group_id);

        const std::string &request_job_order_id = request->job_order_id;
        this->robot_task_->set__job_order_id(request_job_order_id);

        RCLCPP_INFO(
            this->node_->get_logger(),
            "{%s} request cb\n\tjob_group : [%s]\n\tjob_kind : [%s]\n\tjob_plan_id : [%s]\n\tjob_group_id : [%s]\n\tjob_order_id : [%s]",
            RCL_REGISTER_TASK_SERVICE_SERVER_NAME,
            this->robot_task_->get__job_group().c_str(),
            this->robot_task_->get__job_kind().c_str(),
            this->robot_task_->get__job_plan_id().c_str(),
            this->robot_task_->get__job_group_id().c_str(),
            this->robot_task_->get__job_order_id().c_str());
        RCLCPP_LINE_INFO();

        response->set__result_code(RCL_REGISTER_TASK_REGISTER_KEY_SUCCESS_CODE);
        response->set__result_message(RCL_REGISTER_TASK_REGISTER_KEY_SUCCESS_MESSAGE);
    }
}
#ifndef NAVIGATION_STATUS_PUBLISHER__HXX
#define NAVIGATION_STATUS_PUBLISHER__HXX

#include "navigation_status_publisher/navigation_status_publisher_utils.hxx"

namespace robot_status_publisher
{
    namespace navigation
    {
        class NavigationStatus final
        {
        private:
            int8_t status_code_;
            std::string start_time_;
            std::string end_time_;
            float start_battery_level_;
            float end_battery_level_;
            double start_dist_;
            double end_dist_;
        public:
            explicit NavigationStatus();
            virtual ~NavigationStatus();   
            int8_t get__status_code();
            void set__status_code(int8_t status_code);
            std::string get__start_time();
            void set__start_time(std::string start_time);
            std::string get__end_time();
            void set__end_time(std::string end_time);
            float get__start_battery_level();
            void set__start_battery_level(float start_battery_level);
            float get__end_battery_level();
            void set__end_battery_level(float end_battery_level);
            double get__start_dist();
            void set__start_dist(double start_dist);
            double get__end_dist();
            void set__end_dist(double end_dist); 
        };

        class NavigationStatusPublisher final
        {
        private:
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<robot_status_publisher::navigation::NavigationStatus> navigation_status_;

            sensor_msgs::msg::BatteryState::SharedPtr battery_state_;

            rclcpp::CallbackGroup::SharedPtr battery_state_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;

            robot_status_msgs::msg::VelocityStatus::SharedPtr velocity_state_;

            rclcpp::CallbackGroup::SharedPtr velocity_state_subscription_cb_group_;
            rclcpp::Subscription<robot_status_msgs::msg::VelocityStatus>::SharedPtr velocity_state_subscription_;

            rclcpp::CallbackGroup::SharedPtr gts_navigation_status_subscription_cb_group_;
            rclcpp::Subscription<gts_navigation_msgs::msg::NavigationStatusStamped>::SharedPtr gts_navigation_status_subscription_;

            rclcpp::CallbackGroup::SharedPtr gts_navigation_result_subscription_cb_group_;
            rclcpp::Subscription<gts_navigation_msgs::msg::NavigationResultStamped>::SharedPtr gts_navigation_result_subscription_;

            rclcpp::CallbackGroup::SharedPtr navigation_task_status_publisher_cb_group_;
            rclcpp::Publisher<robot_status_msgs::msg::NavigationStatus>::SharedPtr navigation_status_publisher_;

            void flag_rcl_connections(const char *connection_type, const char *connection_name);
            void battery_state_subscription_cb(sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb);
            bool check_battery_state();
            void velocity_state_subscription_cb(robot_status_msgs::msg::VelocityStatus::SharedPtr velocity_state_cb);
            bool check_velocity_state();
            void gts_navigation_status_subscription_cb(gts_navigation_msgs::msg::NavigationStatusStamped::SharedPtr gts_navigation_status_cb);
            void gts_navigation_result_subscription_cb(gts_navigation_msgs::msg::NavigationResultStamped::SharedPtr gts_navigation_result_cb);
            void navigation_task_status_publish(const char *goal_status, const char *job, const char *status);
            std::string capture_current_time();

        public:
            explicit NavigationStatusPublisher(rclcpp::Node::SharedPtr main_node);
            virtual ~NavigationStatusPublisher();
        };
    }
}

#endif
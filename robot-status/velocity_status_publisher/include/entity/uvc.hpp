#ifndef VELOCITY_STATUS_PUBLISHER__ENTITY__UVC__HPP_
#define VELOCITY_STATUS_PUBLISHER__ENTITY__UVC__HPP_
#include "rclcpp/rclcpp.hpp"
namespace ENTITY{
    class UVC{
        private :   
            rclcpp::Time prev_time_;
            double prev_velocity_;
            double accumulate_distance_;
        public:
            UVC(){
                
                prev_velocity_ = 0;
                accumulate_distance_ = 0;
            }
            void set_prev_time(rclcpp::Time time){
                prev_time_ = time;
            }
            void set_perv_velocity(double velocity){
                prev_velocity_ = velocity;
            }
            void add_distance(double accumlate_distance){
                this->accumulate_distance_+=accumlate_distance;
            }
            rclcpp::Time get_prev_time(){
                return prev_time_;
            }
            double get_prev_velocity(){
                return prev_velocity_;
            }
            double get_acc_distance(){
                return accumulate_distance_;
            }
    };
}
#endif
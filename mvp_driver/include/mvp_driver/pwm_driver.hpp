/*
    This library interfce with PCA9685 Chip for PWM signal generations

*/

#ifndef PWM_DRIVER_HPP_
#define PWM_DRIVER_HPP_


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string>
#include <vector>
#include "PCA9685.h"
#include <stdint.h>

class PwmDriver : public rclcpp::Node
{
    public:
        PwmDriver(std::string name = "pwm_driver");

    
    private:
        std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> thruster_sub;
        std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> servo_sub;
        
        //thruster params
        int m_thruster_num;
        std::vector<std::string> m_thruster_ch_list;
        std::vector<std::string> m_thruster_topic_list;
        std::vector<double> m_thruster_min_us;
        std::vector<double> m_thruster_max_us;

        struct thruster_t{
            int index;
            int channel;
            std::string topic_name;
            int min_us;
            int max_us;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
        };
        std::vector<thruster_t> thrusters;
        
        void f_thruster_callback(const std_msgs::msg::Float64::SharedPtr msg, int i); 

        ///servos
        struct servo_t{
            int index;
            int channel;
            int min_pwm;
            int max_pwm;
            std::string topic_name;
            std::string joint_name;
        };

        
        std::vector<servo_t> servos;

        PCA9685 pca{};
};


#endif
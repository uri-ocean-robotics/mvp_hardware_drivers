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
        PwmDriver(std::string name = "pwm_driver_node");

    
    private:
        double m_pwm_ms_bias;
        
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

        ///led
        struct led_t{
            int index;
            int channel;
            std::string topic_name;
            int min_us;
            int max_us;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
        };
        std::vector<led_t> leds;
        void f_led_callback(const std_msgs::msg::Float64::SharedPtr msg, int i); 

        PCA9685 pca{};
};


#endif
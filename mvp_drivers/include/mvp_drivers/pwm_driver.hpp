/*
    This library interfce with PCA9685 Chip for PWM signal generations

*/

#ifndef PWM_DRIVER_HPP_
#define PWM_DRIVER_HPP_


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "string"
#include "vector"
#include "yaml-cpp/yaml.h"

class PwmDriver : public rclcpp::Node
{
    public:
        PwmDriver(std::string name = "pwm_driver");
        
        //this function parse the YAML file for PWM configuration
        bool f_load_configuration();

        //this function will check if PWM chip is available on i2c bus

        bool f_check_pwm_ic();

        //this will set the pwm signal based on the topics
        void f_set_pwm();
        

    
    private:
        std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> thruster_sub;
        std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> servo_sub;

        std::string m_config_file;
        
        struct thruster_t{
            int channel;
            int min_pwm;
            int max_pwm;
            std::string topic_name;
        };
        
        struct servo_t{
            int channel;
            int min_pwm;
            int max_pwm;
            std::string topic_name;
            std::string joint_name;
        };

        std::vector<thruster_t> thrusters;
        std::vector<servo_t> servos;
};


#endif
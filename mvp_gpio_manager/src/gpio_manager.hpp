/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "memory"
#include <string>
#include <vector>
#include <functional>


class GPIOManager
{
    private:
        ros::NodeHandlePtr m_nh;
        ros::NodeHandlePtr m_pnh;

        std::vector<std::string> m_device_name;
        std::vector<int> m_gpio_id;  //gpio number
        int m_gpio_count;   

        struct gpio_t
        {
            int state;
            int id;
            std::string device_name;
            std::string gpio_num;
            std::string service_name; 
            ros::ServiceServer m_set_gpio;
        };

        std::vector<std::string> m_pwm_device;
        std::vector<int> m_pwm_id;  //pwm number
        int m_pwm_count=0;   

        struct pwm_t
        {
            int state;
            int id;
            std::string device_name;
            std::string gpio_num;
            std::string topic_name; 
            ros::Subscriber m_set_pwm;
        };


        int m_pwm_clock;
        int m_pwm_range;
        std::vector<gpio_t> gpio_vector;
        std::vector<pwm_t> pwm_vector;

        ros::ServiceServer m_get_p_state;
        ros::ServiceServer m_set_all_p_state;

        bool f_cb_srv_set_power(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res, int id);

        bool f_cb_srv_set_power_all(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res);
        
        bool f_cb_srv_get_state(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res);
                           
        bool f_set_gpio_value(std::string value, std::string gpio_name);

        void f_cb_set_pwm(const std_msgs::Float64ConstPtr& msg, int id);

        bool f_initialize_gpio();

    public:
        GPIOManager();
};
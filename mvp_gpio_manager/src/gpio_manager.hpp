/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "memory"
#include <string>
#include <vector>
#include <functional>


class GPIOManager : public rclcpp::Node
{
    public:
        GPIOManager(std::string name = "gpio_manager");

    private:
        

        std::vector<std::string> m_device_name;
        std::vector<long int> m_gpio_id;  //gpio number
        int m_gpio_count;   

        struct gpio_t
        {
            int state;
            int id;
            std::string device_name;
            std::string gpio_num;
            std::string service_name; 
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_set_gpio;
        };

        std::vector<gpio_t> gpio_vector;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_get_p_state;

        bool f_cb_srv_set_power(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> response, int id);
        
        bool f_cb_srv_get_state(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
                           

        bool f_initialize_gpio();

    
};
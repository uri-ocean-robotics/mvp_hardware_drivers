
#include "mvp_driver/pwm_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "cstdio"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

PwmDriver::PwmDriver(std::string name) : Node(name)
{
    this->declare_parameter("thruster_num", m_thruster_num);
    this->get_parameter("thruster_num", m_thruster_num);


    this->declare_parameter("thruster_ch_list", m_thruster_ch_list);
    this->get_parameter("thruster_ch_list", m_thruster_ch_list);

    this->declare_parameter("thruster_topic_list", m_thruster_topic_list);
    this->get_parameter("thruster_topic_list", m_thruster_topic_list);

    this->declare_parameter("thruster_min_us", m_thruster_min_us);
    this->get_parameter("thruster_min_us", m_thruster_min_us);

    this->declare_parameter("thruster_max_us", m_thruster_max_us);
    this->get_parameter("thruster_max_us", m_thruster_max_us);

    pca.set_pwm_freq(50.0);

    for (int i =0; i< m_thruster_num; i++)
    {
        thruster_t t; 
        t.index = i;
        t.channel = std::stoi(m_thruster_ch_list[i]);
        t.topic_name = m_thruster_topic_list[i];
        t.min_us = m_thruster_min_us[i];
        t.max_us = m_thruster_max_us[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 
                                                                   10, 
                                                                   [this, i](const std_msgs::msg::Float64::SharedPtr msg){
                                                                    this->f_thruster_callback(msg, i);
                                                                    }
                                                                    );
        pca.set_pwm_ms(t.channel, 1.5);
        thrusters.push_back(t);
    }
}

void PwmDriver::f_thruster_callback(const std_msgs::msg::Float64::SharedPtr msg, int i)
{
    //scale it

    float a = (thrusters[i].max_us - thrusters[i].min_us)/2.0;
    float b = (thrusters[i].max_us + thrusters[i].min_us)/2.0;
    //pwm = a *msg->data + b
    // b= (min+max)/2
    // a = (max-min)/2
    int u = (int) (a * msg->data + b);

    pca.set_pwm_ms(thrusters[i].channel, u);

}
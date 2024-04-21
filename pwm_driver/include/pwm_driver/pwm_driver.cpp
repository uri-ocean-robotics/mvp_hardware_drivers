
#include "pwm_driver/pwm_driver.hpp"
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
    //thruster params
    int m_thruster_num;
    std::vector<long int> m_thruster_ch_list;
    std::vector<std::string> m_thruster_topic_list;
    std::vector<long int> m_thruster_min_us;
    std::vector<long int> m_thruster_max_us;
    double m_pwm_frequency;
    

    this->declare_parameter("pwm_frequency", m_pwm_frequency);
    this->get_parameter("pwm_frequency", m_pwm_frequency);

    this->declare_parameter("pwm_ms_bias", m_pwm_ms_bias);
    this->get_parameter("pwm_ms_bias", m_pwm_ms_bias);

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

    pca.set_pwm_freq(m_pwm_frequency);

    for (int i =0; i< m_thruster_num; i++)
    {
        thruster_t t; 
        t.index = i;
        t.channel = m_thruster_ch_list[i];
        t.topic_name = m_thruster_topic_list[i];
        printf("%s\r\n", t.topic_name.c_str());
        t.min_us = m_thruster_min_us[i];
        t.max_us = m_thruster_max_us[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 
                                                                   10, 
                                                                   [this, i](const std_msgs::msg::Float64::SharedPtr msg){
                                                                    this->f_thruster_callback(msg, i);
                                                                    }
                                                                    );
        pca.set_pwm_ms(t.channel, 1.5 + m_pwm_ms_bias);
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
    double u = (a * msg->data + b)/1000.0 + m_pwm_ms_bias;
    printf("ch=%d, pwm=%lf\r\n",thrusters[i].channel, u-m_pwm_ms_bias);

    pca.set_pwm_ms(thrusters[i].channel, u);

}
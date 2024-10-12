
#include "pwm_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "cstdio"
#include <chrono>
#include <functional>
#include <memory>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

PwmDriver::PwmDriver(std::string name) : Node(name)
{
    const char *i2c_filename = "/dev/i2c-1";
    if ((i2c_file_ = open(i2c_filename, O_RDWR)) < 0)
    {
        perror("Failed to open the i2c bus");
        return;
    }

    double m_pwm_frequency;
    this->declare_parameter("pwm_frequency", m_pwm_frequency);
    this->get_parameter("pwm_frequency", m_pwm_frequency);

    this->declare_parameter("pwm_ms_bias", m_pwm_ms_bias);
    this->get_parameter("pwm_ms_bias", m_pwm_ms_bias);

    pca.set_pwm_freq(m_pwm_frequency);

    //thruster params
    int m_thruster_num;
    
    std::vector<std::string> m_thruster_topic_list;
    std::vector<long int> m_thruster_min_us;
    std::vector<long int> m_thruster_max_us;
    

   
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

    this->declare_parameter("thruster_init_us", m_thruster_init_us);
    this->get_parameter("thruster_init_us", m_thruster_init_us);

    //led params
    std::vector<long int> m_led_ch_list;
    std::vector<std::string> m_led_topic_list;
    std::vector<long int> m_led_min_us;
    std::vector<long int> m_led_max_us;
    std::vector<long int> m_led_init_us;

    this->declare_parameter("led_ch_list", m_led_ch_list);
    this->get_parameter("led_ch_list", m_led_ch_list);

    this->declare_parameter("led_topic_list", m_led_topic_list);
    this->get_parameter("led_topic_list", m_led_topic_list);

    this->declare_parameter("led_min_us", m_led_min_us);
    this->get_parameter("led_min_us", m_led_min_us);

    this->declare_parameter("led_max_us", m_led_max_us);
    this->get_parameter("led_max_us", m_led_max_us);

    this->declare_parameter("led_init_us", m_led_init_us);
    this->get_parameter("led_init_us", m_led_init_us);

    //declare subscriptions
    for (int i =0; i< (int)m_thruster_ch_list.size(); i++)
    {
        thruster_t t; 
        t.index = i;
        t.channel = m_thruster_ch_list[i];
        t.topic_name =  m_thruster_topic_list[i];
        t.min_us = m_thruster_min_us[i];
        t.max_us = m_thruster_max_us[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 
                                                                   10, 
                                                                   [this, i](const std_msgs::msg::Float64::SharedPtr msg){
                                                                    this->f_thruster_callback(msg, i);
                                                                    }
                                                                    );
        pca.set_pwm_ms(t.channel, m_thruster_init_us[i]/1000.00 + m_pwm_ms_bias);
        thrusters.push_back(t);
    }

    for (int i =0; i< (int)m_led_ch_list.size(); i++)
    {
        led_t t; 
        t.index = i;
        t.channel = m_led_ch_list[i];
        t.topic_name =  m_led_topic_list[i];
        t.min_us = m_led_min_us[i];
        t.max_us = m_led_max_us[i];
        t.sub_ = this->create_subscription<std_msgs::msg::Float64>(t.topic_name, 
                                                                   10, 
                                                                   [this, i](const std_msgs::msg::Float64::SharedPtr msg){
                                                                    this->f_led_callback(msg, i);
                                                                    }
                                                                    );
        pca.set_pwm_ms(t.channel, m_led_init_us[i]/1000.0 + m_pwm_ms_bias);
        leds.push_back(t);
    }


}


void PwmDriver::exit()
{
    // running_ = false;

    // Set all used pwm channel to initial value on exit
    for (int i = 0; i < m_thruster_ch_list.size(); i++)
    {
        thruster_t t;
        t.index = i;
        t.channel = m_thruster_ch_list[i];
        pca.set_pwm_ms(t.channel, m_thruster_init_us[i] / 1000.0 + m_pwm_ms_bias);
    }
    close(i2c_file_);
}

void PwmDriver::f_thruster_callback(const std_msgs::msg::Float64::SharedPtr msg, int i)
{
    //scale it
    if(msg->data >= -1.0 && msg->data<=1.0)
    {
    float a = (thrusters[i].max_us - thrusters[i].min_us)/2.0;
    float b = (thrusters[i].max_us + thrusters[i].min_us)/2.0;
    //pwm = a *msg->data + b
    // b= (min+max)/2
    // a = (max-min)/2
    double u = (a * msg->data + b)/1000.0 + m_pwm_ms_bias;
    // printf("ch=%d, pwm=%lf\r\n",thrusters[i].channel, u-m_pwm_ms_bias);

    pca.set_pwm_ms(thrusters[i].channel, u);
    }
    else
    {
        printf("input out of range\r\n");
    }

}


void PwmDriver::f_led_callback(const std_msgs::msg::Float64::SharedPtr msg, int i)
{
   //scale it
    if(msg->data >= 0.0 && msg->data<=1.0)
    {
    float a = (leds[i].max_us - leds[i].min_us);
    float b = leds[i].min_us;
    //pwm = a *msg->data + b
    // b= min
    // a = max-min
    double u = (a * msg->data + b)/1000.0 + m_pwm_ms_bias;
    // printf("ch=%d, pwm=%lf\r\n",leds[i].channel, u-m_pwm_ms_bias);

    pca.set_pwm_ms(leds[i].channel, u);
    }
    else
    {
        printf("input out of range\r\n");
    }

}
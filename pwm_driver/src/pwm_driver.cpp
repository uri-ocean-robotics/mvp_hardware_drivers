/*
 * PwmDriver.cpp
 * 
 * Author: Farhang Naderi
 * Email: farhang.naderi@uri.edu
 * License: MIT License
 * Year: 2024
 * 
 * Description:
 * This code implements the PwmDriver class, which controls the power to a servo rail and LEDs
 * based on commands received via ROS topics. It also sends heartbeat signals to an ATtiny85
 * microcontroller via I2C to ensure the system remains active and safe.
 */

#include "pwm_driver/pwm_driver.hpp"
#include <cstdio>
#include <thread>
#include <atomic>
// Include I2C libraries
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Define the I2C address of the ATtiny85
#define ATTINY85_I2C_ADDRESS 0x08

PwmDriver::PwmDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), running_(true)
{
    // Initialize I2C
    const char *i2c_filename = "/dev/i2c-1";
    if ((i2c_file_ = open(i2c_filename, O_RDWR)) < 0)
    {
        perror("Failed to open the i2c bus");
        return;
    }

    // if (ioctl(i2c_file_, I2C_SLAVE, ATTINY85_I2C_ADDRESS) < 0)
    // {
    //     perror("Failed to acquire bus access and/or talk to slave");
    //     return;
    // }

    // Initialize heartbeat thread
    // heartbeat_thread_ = std::thread(&PwmDriver::send_heartbeat, this);

    double m_pwm_frequency;
    pnh_.param("pwm_frequency", m_pwm_frequency, 50.0);
    pnh_.param("pwm_ms_bias", m_pwm_ms_bias, 0.0);
    pnh_.param("no_cmd_timeout", m_no_cmd_timeout, 3.0);

    pca.set_pwm_freq(m_pwm_frequency);

    // Thruster params
    // int m_thruster_num;
    std::vector<int> m_thruster_ch_list;
    std::vector<std::string> m_thruster_topic_list;
    std::vector<int> m_thruster_min_us;
    std::vector<int> m_thruster_max_us;
    std::vector<int> m_thruster_init_us;
    std::vector<int> m_thruster_direction((6, 0));

    // nh_.param("thruster_num", m_thruster_num, 8);
    pnh_.getParam("thruster_ch_list", m_thruster_ch_list);
    pnh_.getParam("thruster_topic_list", m_thruster_topic_list);
    pnh_.getParam("thruster_min_us", m_thruster_min_us);
    pnh_.getParam("thruster_max_us", m_thruster_max_us);
    pnh_.getParam("thruster_init_us", m_thruster_init_us);
    pnh_.getParam("thruster_direction", m_thruster_direction);

    // LED params
    std::vector<int> m_led_ch_list;
    std::vector<std::string> m_led_topic_list;
    std::vector<int> m_led_min_us;
    std::vector<int> m_led_max_us;
    std::vector<int> m_led_init_us;

    pnh_.getParam("led_ch_list", m_led_ch_list);
    pnh_.getParam("led_topic_list", m_led_topic_list);
    pnh_.getParam("led_min_us", m_led_min_us);
    pnh_.getParam("led_max_us", m_led_max_us);
    pnh_.getParam("led_init_us", m_led_init_us);

    // Servo params
    std::vector<int> m_servo_ch_list;
    std::vector<std::string> m_servo_topic_list;
    std::vector<int> m_servo_min_us;
    std::vector<int> m_servo_max_us;
    std::vector<int> m_servo_center_us;

    pnh_.getParam("servo_ch_list", m_servo_ch_list);
    pnh_.getParam("servo_topic_list", m_servo_topic_list);
    pnh_.getParam("servo_min_us", m_servo_min_us);
    pnh_.getParam("servo_max_us", m_servo_max_us);
    pnh_.getParam("servo_center_us", m_servo_center_us);

    // Declare subscriptions for thrusters
    ROS_INFO("### Initializing Thrusters. number = %d", m_thruster_ch_list.size());
    for (int i = 0; i < m_thruster_ch_list.size(); i++)
    {
        thruster_t t;
        t.index = i;
        t.channel = m_thruster_ch_list[i];
        t.topic_name = m_thruster_topic_list[i];
        t.min_us = m_thruster_min_us[i];
        t.max_us = m_thruster_max_us[i];
        t.direction = m_thruster_direction[i];
        thruster_subs_.push_back(nh_.subscribe<std_msgs::Float64>(t.topic_name, 10, boost::bind(&PwmDriver::f_thruster_callback, this, _1, i)));
        pca.set_pwm_ms(t.channel, 0);
        sleep(1);
        pca.set_pwm_ms(t.channel, m_thruster_init_us[i] / 1000.0 + m_pwm_ms_bias);
        sleep(1);
        thrusters.push_back(t);
    }
    ROS_INFO("### Thrusters PWM initialization finished", m_thruster_ch_list.size());

    // Declare subscriptions for LEDs
    for (int i = 0; i < m_led_ch_list.size(); i++)
    {
        led_t t;
        t.index = i;
        t.channel = m_led_ch_list[i];
        t.topic_name = m_led_topic_list[i];
        t.min_us = m_led_min_us[i];
        t.max_us = m_led_max_us[i];
        led_subs_.push_back(nh_.subscribe<std_msgs::Float64>(t.topic_name, 10, boost::bind(&PwmDriver::f_led_callback, this, _1, i)));
        pca.set_pwm_ms(t.channel, m_led_init_us[i] / 1000.0 + m_pwm_ms_bias);
        leds.push_back(t);
    }

    // Declare subscriptions for servos
    for (int i = 0; i < m_servo_ch_list.size(); i++)
    {
        servo_t t;
        t.index = i;
        t.channel = m_servo_ch_list[i];
        t.topic_name = m_servo_topic_list[i];
        t.min_us = m_servo_min_us[i];
        t.max_us = m_servo_max_us[i];
        t.center_us = m_servo_center_us[i];
        servo_subs_.push_back(nh_.subscribe<std_msgs::Float64>(t.topic_name, 10, boost::bind(&PwmDriver::f_servo_callback, this, _1, i)));
        pca.set_pwm_ms(t.channel, t.center_us / 1000.0 + m_pwm_ms_bias);
        servos.push_back(t);
    }

    // Initialize the safety timer
    // safety_timer_ = nh_.createTimer(ros::Duration(0.5), &PwmDriver::safety_check, this);  // Check every 0.5 seconds
    last_command_time_ = ros::Time::now().toSec();
}

PwmDriver::~PwmDriver()
{
    running_ = false;
    if (heartbeat_thread_.joinable())
    {
        heartbeat_thread_.join();
    }
    close(i2c_file_);
}

void PwmDriver::send_heartbeat()
{
    while (running_)
    {
        char heartbeat = 'H';
        if (write(i2c_file_, &heartbeat, 1) != 1)
        {
            perror("Failed to write to the i2c bus");
        }
        else
        {
            printf("Heartbeat sent\n");
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void PwmDriver::f_thruster_callback(const std_msgs::Float64::ConstPtr& msg, int i)
{
    // Scale it
    if (msg->data >= -1.0 && msg->data <= 1.0)
    {
        float a = (thrusters[i].max_us - thrusters[i].min_us) / 2.0;
        float b = (thrusters[i].max_us + thrusters[i].min_us) / 2.0;
        double u = (a * thrusters[i].direction*msg->data + b) / 1000.0 + m_pwm_ms_bias;
        // printf("ch=%d, pwm=%lf\r\n", thrusters[i].channel, u - m_pwm_ms_bias);
        pca.set_pwm_ms(thrusters[i].channel, u);
        last_command_time_ = ros::Time::now().toSec();  // Update the last command time
    }
    else
    {
        printf("input out of range\r\n");
    }
}

void PwmDriver::f_led_callback(const std_msgs::Float64::ConstPtr& msg, int i)
{
    // Scale it
    if (msg->data >= 0.0 && msg->data <= 1.0)
    {
        float a = (leds[i].max_us - leds[i].min_us);
        float b = leds[i].min_us;
        double u = (a * msg->data + b) / 1000.0 + m_pwm_ms_bias;
        printf("ch=%d, pwm=%lf\r\n", leds[i].channel, u - m_pwm_ms_bias);
        pca.set_pwm_ms(leds[i].channel, u);
        last_command_time_ = ros::Time::now().toSec();  // Update the last command time
    }
    else
    {
        printf("input out of range\r\n");
    }
}

void PwmDriver::f_servo_callback(const std_msgs::Float64::ConstPtr& msg, int i)
{
    // Scale it
    if (msg->data >= -1.0 && msg->data <= 1.0)
    {
        float a = (servos[i].max_us - servos[i].min_us) / 2.0;
        float b = (servos[i].max_us + servos[i].min_us) / 2.0;
        double u = (a * msg->data + b) / 1000.0 + m_pwm_ms_bias;
        printf("ch=%d, pwm=%lf\r\n", servos[i].channel, u - m_pwm_ms_bias);
        pca.set_pwm_ms(servos[i].channel, u);
        last_command_time_ = ros::Time::now().toSec();  // Update the last command time
    }
    else
    {
        printf("input out of range\r\n");
    }
}

void PwmDriver::safety_check(const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - last_command_time_ > m_no_cmd_timeout)
    {
        // Set servos to neutral position
        for (const auto& servo : servos)
        {
            pca.set_pwm_ms(servo.channel, servo.center_us / 1000.0 + m_pwm_ms_bias);
        }
        printf("Safety check: Setting servos to neutral position\n");
    }
}
/*
    This library interfaces with the PCA9685 Chip for PWM signal generation
*/
#ifndef PWM_DRIVER_HPP_
#define PWM_DRIVER_HPP_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string>
#include <vector>
#include "PCA9685.h"
#include <stdint.h>
#include <thread>
#include <atomic>

class PwmDriver
{
public:
    PwmDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~PwmDriver();

private:
    ros::NodeHandle nh_; 
    ros::NodeHandle pnh_;
    double m_pwm_ms_bias;

    ros::Timer safety_timer_;  // Timer to check for command updates
    ros::Duration timeout_duration_;  // Duration after which to set PWM to neutral

    double m_no_cmd_timeout;
    double last_command_time_;
    std::vector<int> m_thruster_ch_list;
    std::vector<int> m_thruster_init_us;
    void safety_check(const ros::TimerEvent& event);  // Function to check for command updates

    struct thruster_t
    {
        int index;
        int channel;
        std::string topic_name;
        int min_us;
        int max_us;
        int direction;
        ros::Subscriber sub_;
    };
    std::vector<thruster_t> thrusters;
    std::vector<ros::Subscriber> thruster_subs_; 
    void f_thruster_callback(const std_msgs::Float64::ConstPtr& msg, int i); 

    struct servo_t
    {
        int index;
        int channel;
        int min_us;
        int max_us;
        int center_us;
        std::string topic_name;
    };
    std::vector<servo_t> servos;
    std::vector<ros::Subscriber> servo_subs_;
    void f_servo_callback(const std_msgs::Float64::ConstPtr& msg, int i);

    struct led_t
    {
        int index;
        int channel;
        std::string topic_name;
        int min_us;
        int max_us;
        ros::Subscriber sub_;
    };
    std::vector<led_t> leds;
    std::vector<ros::Subscriber> led_subs_;
    void f_led_callback(const std_msgs::Float64::ConstPtr& msg, int i); 

    PCA9685 pca{};

    // Add new members for I2C and heartbeat
    std::thread heartbeat_thread_;
    std::atomic<bool> running_;
    int i2c_file_;
    void send_heartbeat();
};

#endif  // PWM_DRIVER_HPP_
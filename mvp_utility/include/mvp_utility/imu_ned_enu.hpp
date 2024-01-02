/*
    This node converts pressure reading into pose with covariance stamped message for robot localization

*/

#ifndef IMU_NED_ENU_HPP_
#define IMU_NED_ENU_HPP_


#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/imu.hpp"


#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"


class IMUNedEnu : public rclcpp::Node
{
    public:
        IMUNedEnu(std::string name = "imu_ned_to_enu");
    
    private:
        std::string m_frame_id;

        double roll_offset;
        double pitch_offset;
        double yaw_offset;
        
        double roll_reverse;
        double pitch_reverse;
        double yaw_reverse;

        void f_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);        
        
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_out;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_in;
};

#endif
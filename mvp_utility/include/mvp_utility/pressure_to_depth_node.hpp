/*
    This node converts pressure reading into pose with covariance stamped message for robot localization

*/

#ifndef PRESSURE_TO_DEPTH_NODE_HPP_
#define PRESSURE_TO_DEPTH_NODE_HPP_


#include "rclcpp/rclcpp.hpp"
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"


class PressureToDepthNode : public rclcpp::Node
{
    public:
        PressureToDepthNode(std::string name = "pressure_to_depth_node");
    
    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _publisher;
        rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr _subscriber;
        void f_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
        std::string m_frame_id;
        float m_fluid_density;
};

#endif
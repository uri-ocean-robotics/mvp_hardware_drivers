#include "mvp_utility/pressure_to_depth_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

PressureToDepthNode::PressureToDepthNode(std::string name) : Node(name)
{

    //parameter
    this->declare_parameter("fluid_density", 1023.0);
    this->declare_parameter("frame_id", "world");

    this->get_parameter("fluid_density",m_fluid_density);
    // m_frame_id = this->get_parameter("frame_id").as_string();
    this->get_parameter("frame_id",m_frame_id);
    //topic with namespace
    _publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("depth", 10);
    //topci without namespace
    _subscriber = this->create_subscription<sensor_msgs::msg::FluidPressure>("pressure", 10, 
                                                                std::bind(&PressureToDepthNode::f_pressure_callback, 
                                                                this, _1));


}

void PressureToDepthNode::f_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
    geometry_msgs::msg::PoseWithCovarianceStamped depth;
    // depth = geometry_msgs::msg::pose_with_covariance_stamped();
    depth.header = msg->header;
    depth.header.frame_id = m_frame_id;
    depth.pose.pose.position.z = msg->fluid_pressure / (m_fluid_density*9.81);
    
    _publisher->publish(depth);

}
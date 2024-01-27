#include "mvp_utility/imu_ned_enu.hpp"
#include "Eigen/Dense"
#include "iostream"
#include "cstdio"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

IMUNedEnu::IMUNedEnu(std::string name) : Node(name)
{

    //parameter
    this->declare_parameter("roll_offset", 3.1415926);
    this->declare_parameter("pitch_offset", 0.0);
    this->declare_parameter("yaw_offset", 1.5707);
    this->declare_parameter("roll_reverse", 1.0);
    this->declare_parameter("pitch_reverse", -1.0);
    this->declare_parameter("yaw_reverse", -1.0);
    this->declare_parameter("frame_id", "imu");

    //link parameter and variables.
    this->get_parameter("roll_offset",roll_offset);
    this->get_parameter("pitch_offset",pitch_offset);
    this->get_parameter("yaw_offset",yaw_offset);
    this->get_parameter("roll_reverse",roll_reverse);
    this->get_parameter("pitch_reverse",pitch_reverse);
    this->get_parameter("yaw_reverse",yaw_reverse);

    this->get_parameter("frame_id",m_frame_id);

    // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //topic with namespace
    m_imu_out = this->create_publisher<sensor_msgs::msg::Imu>("imu_out/data", 10);
    //topci without namespace
    m_imu_in = this->create_subscription<sensor_msgs::msg::Imu>("imu_in/data", 10, 
                                                                std::bind(&IMUNedEnu::f_imu_callback, 
                                                                this, _1));

}

void IMUNedEnu::f_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // geometry_msgs::msg::TransformStamped t;
    // rclcpp::Time now = this->get_clock()->now();

    // std::string fromFrameRel = "mvp2_test_robot/cg_link";
    // std::string toFrameRel = "mvp2_test_robot/world_ned";

    // try {
    //     t = tf_buffer_->lookupTransform(
    //     toFrameRel, fromFrameRel,
    //     tf2::TimePointZero,
    //     10ms
    //     );
    // } catch (const tf2::TransformException & ex) {
    //     RCLCPP_INFO(
    //     this->get_logger(), "Could not transform %s to %s: %s",
    //     toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    //     return;
    // }

     tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf2::Quaternion newq;
    // newq.setRPY(roll-M_PI, -pitch, -yaw + M_PI_2); default?
    // newq.setRPY(roll, -pitch, -yaw + M_PI_2);
    newq.setRPY(roll*roll_reverse   + roll_offset, 
                pitch*pitch_reverse + pitch_offset,
                yaw*yaw_reverse     + yaw_offset);

    newq = newq.normalize();


    sensor_msgs::msg::Imu m = *msg;
 
    m.orientation.x = newq.x();
    m.orientation.y = newq.y();
    m.orientation.z = newq.z();
    m.orientation.w = newq.w();
    m.header.frame_id = m_frame_id;
    m_imu_out->publish(m);

}
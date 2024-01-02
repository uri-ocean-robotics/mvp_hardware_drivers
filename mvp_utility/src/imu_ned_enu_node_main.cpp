
#include "rclcpp/rclcpp.hpp"

#include "mvp_utility/imu_ned_enu.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<IMUNedEnu> node = std::make_shared<IMUNedEnu>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

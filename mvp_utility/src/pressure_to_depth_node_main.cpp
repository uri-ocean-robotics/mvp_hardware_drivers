
#include "rclcpp/rclcpp.hpp"

#include "mvp_utility/pressure_to_depth_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<PressureToDepthNode> node = std::make_shared<PressureToDepthNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

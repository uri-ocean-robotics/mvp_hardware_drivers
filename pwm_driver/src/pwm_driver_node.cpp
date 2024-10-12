#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include "pwm_driver.hpp"

std::shared_ptr<PwmDriver> node = std::make_shared<PwmDriver>();

void signalHandler(int signum)
{
    RCLCPP_INFO(node->get_logger(), "Interrupt signal (%d) received. Shutting down.", signum);
    node->exit();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<PwmDriver> node = std::make_shared<PwmDriver>();

  std::signal(SIGINT, signalHandler);

  rclcpp::spin(node);

  // rclcpp::shutdown();
  return 0;
}
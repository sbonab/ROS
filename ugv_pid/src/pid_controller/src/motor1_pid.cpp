#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pid_controller/PIDCtrl.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDCtrl>("motor1_pid", "motor1_w_s", "motor1_w_m", "motor1_voltage"));
  rclcpp::shutdown();
  return 0;
}

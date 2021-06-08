#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pid_controller/PIDCtrl.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDCtrl>("motor2_pid", "motor2_w_s", "motor2_w_m", "motor2_voltage"));
  rclcpp::shutdown();
  return 0;
}

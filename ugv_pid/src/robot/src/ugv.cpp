#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/num.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class UGV : public rclcpp::Node
{
  public:
    UGV()
    : Node("ugv")
    {
      w_meas_pub_ = this->create_publisher<interfaces::msg::Num>("motor_w_m", 10);
      w_set_pub_ = this->create_publisher<interfaces::msg::Num>("motor_w_s", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&UGV::timer_cb, this));
    }

  private:
    void timer_cb()
    {
      auto msg_w_m = interfaces::msg::Num();
      msg_w_m.num = -1.0;
      RCLCPP_INFO(this->get_logger(), "Publishing Measured W: %f", msg_w_m.num);
      w_meas_pub_->publish(msg_w_m);
      
      auto msg_w_s = interfaces::msg::Num();
      msg_w_s.num = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing Set W: %f", msg_w_s.num);
      w_set_pub_->publish(msg_w_s);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::Num>::SharedPtr w_meas_pub_;
    rclcpp::Publisher<interfaces::msg::Num>::SharedPtr w_set_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UGV>());
  rclcpp::shutdown();
  return 0;
}
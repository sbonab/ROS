#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class CMD : public rclcpp::Node
{
  public:
    CMD()
    : Node("set_cmd")
    {
      twist_sub_   = this->create_subscription<geometry_msgs::msg::Twist>("ugv/cmd_vel", 10, std::bind(&CMD::convert_cb, this, _1));
      w_set_1_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor1_w_s", 10);
      w_set_2_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor2_w_s", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&CMD::timer_cb, this));
      t_ = 0.0;
    }

  private:
    void timer_cb()
    {
      
      RCLCPP_INFO(this->get_logger(), "Publishing set w_1: %f", w_1_.data);
      w_set_1_pub_->publish(w_1_);
      
      RCLCPP_INFO(this->get_logger(), "Publishing set w_2: %f", w_2_.data);
      w_set_2_pub_->publish(w_2_);

    }

    void convert_cb(const geometry_msgs::msg::Twist::SharedPtr twist){
      double vel_gain{300};
      double rot_gain{100};
      w_1_.data = vel_gain * twist->linear.x - rot_gain * twist->angular.z;
      w_2_.data = vel_gain * twist->linear.x + rot_gain * twist->angular.z;
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr w_set_1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr w_set_2_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    std_msgs::msg::Float64 w_1_;
    std_msgs::msg::Float64 w_2_;
    double t_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CMD>());
  rclcpp::shutdown();
  return 0;
}
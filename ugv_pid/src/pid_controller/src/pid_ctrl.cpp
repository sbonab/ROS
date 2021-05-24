#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/num.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PIDCtrl : public rclcpp::Node
{
  public:
    PIDCtrl()
    : Node("pid_ctrl")
    {
      w_meas_sub_ = this->create_subscription<interfaces::msg::Num>(
      "motor_w_m", 10, std::bind(&PIDCtrl::w_meas_cb, this, _1));
      w_set_sub_ = this->create_subscription<interfaces::msg::Num>(
      "motor_w_s", 10, std::bind(&PIDCtrl::w_set_cb, this, _1));
      voltage_pub_ = this->create_publisher<interfaces::msg::Num>("motor_voltage", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PIDCtrl::timer_cb, this));

      Kp_ = 1.0;
      Ki_ = 0.0;
      Kd_ = 0.0;

      p_error_ = 0.0;
      i_error_ = 0.0;
      d_error_ = 0.0;

    }

  private:
    void timer_cb()
    {
      auto message = interfaces::msg::Num();
      message.num = CalcCMD();
      RCLCPP_INFO(this->get_logger(), "Publishing: %f", message.num);
      voltage_pub_->publish(message);
    }
    
    void w_meas_cb(const interfaces::msg::Num::SharedPtr w)
    {
      w_meas_ = w->num;
    }

    void w_set_cb(const interfaces::msg::Num::SharedPtr w)
    {
      w_set_ = w->num;
    }
    
    void UpdateError(double e)
    {
      d_error_ = e - p_error_;
      i_error_ += e;
      p_error_ = e;
    }

    double CalcCMD()
    {
      double e = w_set_ - w_meas_;
      UpdateError(e);

      double u = Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
      return u;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::Num>::SharedPtr voltage_pub_;
    rclcpp::Subscription<interfaces::msg::Num>::SharedPtr w_meas_sub_;
    rclcpp::Subscription<interfaces::msg::Num>::SharedPtr w_set_sub_;
    
    double w_meas_;
    double w_set_;

    /**
    * PID Coefficients
    */
    double Kp_;
    double Ki_;
    double Kd_;

    /**
    * PID Errors
    */
    double p_error_;
    double i_error_;
    double d_error_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDCtrl>());
  rclcpp::shutdown();
  return 0;
}
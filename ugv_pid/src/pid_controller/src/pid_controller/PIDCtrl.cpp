#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>

#include "pid_controller/PIDCtrl.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


    PIDCtrl::PIDCtrl(std::string node_name, std::string set_topic, std::string meas_topic, std::string u_topic)
    : Node(node_name)
    {
      meas_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      meas_topic, 10, std::bind(&PIDCtrl::meas_cb, this, _1));
      set_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      set_topic, 10, std::bind(&PIDCtrl::set_cb, this, _1));
      u_pub_ = this->create_publisher<std_msgs::msg::Float64>(u_topic, 10);
      timer_ = this->create_wall_timer(
      20ms, std::bind(&PIDCtrl::timer_cb, this));

      Kp_ = 0.0;
      Ki_ = 0.01;
      Kd_ = 0.0;

      p_error_ = 0.0;
      i_error_ = 0.0;
      d_error_ = 0.0;

    }

    PIDCtrl::~PIDCtrl(){}

    void PIDCtrl::timer_cb()
    {
      auto cmd = std_msgs::msg::Float64();
      cmd.data = CalcCMD();
      RCLCPP_INFO(this->get_logger(), "Publishing: %f", cmd.data);
      u_pub_->publish(cmd);
    }
    
    void PIDCtrl::meas_cb(const std_msgs::msg::Float64::SharedPtr w)
    {
      meas_ = w->data;
      RCLCPP_INFO(this->get_logger(), "Received measured data: %f", w->data);
    }

    void PIDCtrl::set_cb(const std_msgs::msg::Float64::SharedPtr w)
    {
      set_ = w->data;
      RCLCPP_INFO(this->get_logger(), "Received set data: %f", w->data);
    }
    
    void PIDCtrl::UpdateError(double e)
    {
      d_error_ = e - p_error_;
      i_error_ += e;
      p_error_ = e;
    }

    double PIDCtrl::CalcCMD()
    {
      double e = set_ - meas_;
      UpdateError(e);

      double u = Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
      return u;
    }


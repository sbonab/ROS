#ifndef PIDCTRL_H_
#define PIDCTRL_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PIDCtrl : public rclcpp::Node
{
  public:
    PIDCtrl(std::string node_name, std::string set_topic, std::string meas_topic, std::string u_topic);
    ~PIDCtrl();
  
  private:
    void timer_cb();
    
    void meas_cb(const std_msgs::msg::Float64::SharedPtr w);

    void set_cb(const std_msgs::msg::Float64::SharedPtr w);
    
    void UpdateError(double e);
    
    double CalcCMD();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr u_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr meas_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_sub_;
    
    double meas_;
    double set_;

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

#endif
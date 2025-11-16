#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <atomic>
#include <string>

namespace arm_sensors {

class SafetySupervisor final : public rclcpp::Node {
public:
  SafetySupervisor();

private:
  void on_angle(const std_msgs::msg::Float64::SharedPtr msg);

  std::string angle_topic_;
  std::string pwm_topic_;
  double      limit_deg_{180.0};
  double      limit_rad_{3.141592653589793};
  int         stop_us_{1000};
  bool        shutdown_on_trip_{true};
  bool        latch_trip_{true};

  std::atomic<bool> tripped_{false};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_angle_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr      pub_pwm_;
};

} // namespace arm_sensors

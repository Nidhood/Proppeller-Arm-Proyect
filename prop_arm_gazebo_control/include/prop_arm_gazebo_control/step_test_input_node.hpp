#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <string>

namespace prop_arm_control {

class StepTestInput final : public rclcpp::Node {
public:
  explicit StepTestInput(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~StepTestInput() override;  // destructor agregado

private:
  void on_timer();

  double rate_hz_{100.0};
  double up_time_s_{5.0};
  double down_time_s_{5.0};
  int    us_low_{1272};
  int    us_high_{1324};
  std::string topic_pwm_{"/esc/pwm_us"};

  bool   high_{false};
  rclcpp::Time phase_start_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace prop_arm_control

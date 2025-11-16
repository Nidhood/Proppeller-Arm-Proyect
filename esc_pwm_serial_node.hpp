#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace arm_sensors {

class EscPwmSerialNode : public rclcpp::Node {
public:
  EscPwmSerialNode();
  ~EscPwmSerialNode() override;

private:
  void on_pwm(const std_msgs::msg::UInt16::SharedPtr msg);
  int  pi_{-1};
  int  gpio_{18};
  int  min_us_{1000};
  int  max_us_{2000};
  int  arm_us_{1000};
  int  arm_ms_{2000};
  bool disable_on_shutdown_{true};
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr   pub_fb_;
};

} // namespace arm_sensors

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <optional>
#include <string>
#include <cstdint>

namespace sensors {

class ArmAngleNode final : public rclcpp::Node {
private:
  int fd_{-1};
  std::string dev_;

  std::uint8_t  bus_;
  std::uint8_t  addr_;
  std::uint8_t  reg_;
  std::uint16_t bits_;
  std::uint16_t offset_;
  double rate_;
  double scale_{1.0};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::duration<double> period_{std::chrono::duration<double>(0.001)};

  void on_timer();
  std::optional<uint16_t> read_raw_angle();
  static std::string to_hex(int v);

public:
  ArmAngleNode();
  ~ArmAngleNode() override;

  void set_sample_period_seconds(double seconds);
};

} // namespace sensors

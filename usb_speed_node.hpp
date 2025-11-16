#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <thread>
#include <atomic>

namespace prop_arm_control
{

  class UsbSpeedNode final : public rclcpp::Node
  {
  public:
    explicit UsbSpeedNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());
    ~UsbSpeedNode() override;

  private:
    // params
    std::string dev_{"/dev/ttyUSB0"};
    int baud_{921600};
    int bits_{14};
    std::string topic_{"/prop_arm/motor_speed_est"};

    // serial
    int fd_{-1};
    std::thread reader_;
    std::atomic<bool> run_{false};

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;

    // unwrap/derivative state
    double prev_theta_{0.0};
    bool have_prev_{false};
    uint64_t prev_tus_{0}; // micros from Arduino

    // helpers
    void reader_loop();
    static bool setup_serial(int fd, int baud);
    static speed_t baud_to_flag(int baud);
  };

} // namespace prop_arm_control

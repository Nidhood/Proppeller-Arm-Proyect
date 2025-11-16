#include "arm_sensors/safety_supervisor.hpp"
#include <cmath>

using std_msgs::msg::Float64;
using std_msgs::msg::UInt16;

namespace arm_sensors {

SafetySupervisor::SafetySupervisor() : rclcpp::Node("safety_supervisor") {
  angle_topic_       = this->declare_parameter<std::string>("angle_topic", "/arm/angle_rad");
  pwm_topic_         = this->declare_parameter<std::string>("pwm_topic", "/esc/pwm_us");
  limit_deg_         = this->declare_parameter<double>("limit_deg", 180.0);
  stop_us_           = this->declare_parameter<int>("stop_us", 1000);
  shutdown_on_trip_  = this->declare_parameter<bool>("shutdown_on_trip", true);
  latch_trip_        = this->declare_parameter<bool>("latch_trip", true);

  limit_rad_ = limit_deg_ * M_PI / 180.0;

  auto qos = rclcpp::SensorDataQoS();
  sub_angle_ = this->create_subscription<Float64>(
      angle_topic_, qos, std::bind(&SafetySupervisor::on_angle, this, std::placeholders::_1));

  pub_pwm_ = this->create_publisher<UInt16>(pwm_topic_, 10);
}

void SafetySupervisor::on_angle(const Float64::SharedPtr msg) {
  if (tripped_.load(std::memory_order_relaxed)) return;

  const double a = std::abs(msg->data);
  if (a > limit_rad_) {
    tripped_.store(true, std::memory_order_relaxed);

    UInt16 pwm; pwm.data = static_cast<uint16_t>(stop_us_);
    pub_pwm_->publish(pwm);

    if (shutdown_on_trip_) {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(shared_from_this());
      exec.spin_some();
      rclcpp::shutdown();
    } else if (!latch_trip_) {
      tripped_.store(false, std::memory_order_relaxed);
    }
  }
}

} // namespace arm_sensors

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_sensors::SafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}

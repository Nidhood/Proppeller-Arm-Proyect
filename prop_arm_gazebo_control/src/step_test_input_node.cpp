#include "prop_arm_gazebo_control/step_test_input_node.hpp"
#include <algorithm>
using std_msgs::msg::UInt16;

namespace prop_arm_control {

StepTestInput::StepTestInput(const rclcpp::NodeOptions& opts)
: rclcpp::Node("step_test_input", opts)
{
  rate_hz_     = this->declare_parameter<double>("rate_hz", 100.0);
  up_time_s_   = this->declare_parameter<double>("up_time_s", 5.0);
  down_time_s_ = this->declare_parameter<double>("down_time_s", 5.0);
  us_low_      = this->declare_parameter<int>("us_low", 1272);
  us_high_     = this->declare_parameter<int>("us_high", 1324);
  topic_pwm_   = this->declare_parameter<std::string>("topic_pwm", "/esc/pwm_us");

  rate_hz_     = std::max(1.0, rate_hz_);
  up_time_s_   = std::max(0.01, up_time_s_);
  down_time_s_ = std::max(0.01, down_time_s_);
  us_low_      = std::clamp(us_low_,  1000, 2000);
  us_high_     = std::clamp(us_high_, 1000, 2000);
  if (us_high_ < us_low_) std::swap(us_high_, us_low_);
  us_high_     = std::min(us_high_, 1324);

  pub_ = this->create_publisher<UInt16>(topic_pwm_, 10);

  const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StepTestInput::on_timer, this));

  phase_start_ = this->get_clock()->now();
}

StepTestInput::~StepTestInput() {
  // En el destructor, publica 1000 µs para apagar el ESC
  if (pub_) {
    UInt16 safe;
    safe.data = 1000;
    RCLCPP_WARN(get_logger(), "Nodo finalizado: enviando PWM seguro de 1000 µs");
    pub_->publish(safe);
    // breve espera para garantizar transmisión
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

void StepTestInput::on_timer() {
  const double elapsed = (this->get_clock()->now() - phase_start_).seconds();
  const double phase_len = high_ ? up_time_s_ : down_time_s_;

  if (elapsed >= phase_len) {
    high_ = !high_;
    phase_start_ = this->get_clock()->now();
  }

  UInt16 msg;
  msg.data = static_cast<uint16_t>(high_ ? us_high_ : us_low_);
  pub_->publish(msg);
}

} // namespace prop_arm_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<prop_arm_control::StepTestInput>());
  rclcpp::shutdown();
  return 0;
}

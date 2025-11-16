#include "arm_sensors/esc_pwm_serial.hpp"

#include <pigpiod_if2.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <cstdio>
#include <string>

using namespace std::chrono_literals;

namespace {

int read_pid(const char* cmd) {
  FILE* p = popen(cmd, "r");
  if (!p) return -1;
  char buf[64]{};
  if (!fgets(buf, sizeof(buf), p)) { pclose(p); return -1; }
  pclose(p);
  return std::atoi(buf);
}

bool spawn_pigpiod(int port) {
  std::string cmd = "pigpiod -g -p " + std::to_string(port) + " >/dev/null 2>&1 &";
  return std::system(cmd.c_str()) == 0;
}

bool stop_pigpiod_by_pid(int pid) {
  if (pid <= 0) return false;
  std::string cmd = "kill -TERM " + std::to_string(pid);
  return std::system(cmd.c_str()) == 0;
}

} // anon

namespace arm_sensors {

EscPwmSerialNode::EscPwmSerialNode()
: rclcpp::Node("esc_pwm_serial")
{
  gpio_                = this->declare_parameter<int>("gpio", 18);
  min_us_              = this->declare_parameter<int>("min_us", 1000);
  max_us_              = this->declare_parameter<int>("max_us", 2000);
  arm_us_              = this->declare_parameter<int>("arm_us", 1000);
  arm_ms_              = this->declare_parameter<int>("arm_ms", 2000);
  disable_on_shutdown_ = this->declare_parameter<bool>("disable_on_shutdown", true);

  const int port = 8888;
  int pigpio_pid = read_pid("pgrep -f 'pigpiod.*-p 8888'");
  bool self_started = false;

  pi_ = pigpio_start(nullptr, nullptr);
  if (pi_ < 0) {
    int rc = std::system("pgrep pigpiod >/dev/null || pigpiod -g >/dev/null 2>&1 &");
    (void)rc;
    for (int i = 0; i < 40 && pi_ < 0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      pi_ = pigpio_start(nullptr, nullptr); // default
    }
  }
  if (pi_ < 0) {
    RCLCPP_FATAL(this->get_logger(), "pigpio_start failed");
    throw std::runtime_error("pigpio_start failed");
  }
  if (pi_ < 0) throw std::runtime_error("pigpio_start failed");

  set_mode(pi_, gpio_, PI_OUTPUT);

  if (arm_ms_ > 0) {
    const int arm_pw = std::clamp(arm_us_, min_us_, max_us_);
    set_servo_pulsewidth(pi_, gpio_, arm_pw);
    std::this_thread::sleep_for(std::chrono::milliseconds(arm_ms_));
  }

  pub_fb_ = this->create_publisher<std_msgs::msg::UInt16>("/esc/pwm_us_fb", 10);
  sub_ = this->create_subscription<std_msgs::msg::UInt16>(
    "/esc/pwm_us", 10,
    [this](const std_msgs::msg::UInt16::SharedPtr msg){ this->on_pwm(msg); });

  this->set_on_shutdown([this, self_started, pigpio_pid](){
    if (disable_on_shutdown_ && pi_ >= 0) set_servo_pulsewidth(pi_, gpio_, 0);
    if (pi_ >= 0) { pigpio_stop(pi_); pi_ = -1; }
    if (self_started) stop_pigpiod_by_pid(pigpio_pid);
  });

  RCLCPP_INFO(this->get_logger(),
              "esc_pwm_serial gpio=%d range=%d..%d arm=%d/%dms",
              gpio_, min_us_, max_us_, arm_us_, arm_ms_);
}

EscPwmSerialNode::~EscPwmSerialNode() {
  if (disable_on_shutdown_ && pi_ >= 0) set_servo_pulsewidth(pi_, gpio_, 0);
  if (pi_ >= 0) { pigpio_stop(pi_); pi_ = -1; }
}

void EscPwmSerialNode::on_pwm(const std_msgs::msg::UInt16::SharedPtr msg) {
  if (pi_ < 0) return;
  const unsigned us = static_cast<unsigned>(std::clamp<int>(msg->data, min_us_, max_us_));
  const int rc = set_servo_pulsewidth(pi_, gpio_, us);
  if (rc != 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "set_servo_pulsewidth rc=%d", rc);
  }
  std_msgs::msg::UInt16 fb;
  fb.data = static_cast<uint16_t>(us);
  pub_fb_->publish(fb);
}

} // namespace arm_sensors

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_sensors::EscPwmSerialNode>());
  rclcpp::shutdown();
  return 0;
}

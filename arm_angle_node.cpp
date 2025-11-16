#include "arm_angle_node.hpp"
#include <stdexcept>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

using std_msgs::msg::Float64;

namespace sensors {

std::string ArmAngleNode::to_hex(int v) {
  char tmp[8]{};
  std::snprintf(tmp, sizeof(tmp), "%02X", (v & 0xFF));
  return std::string(tmp);
}

ArmAngleNode::ArmAngleNode()
: Node("arm_angle_publisher", rclcpp::NodeOptions().use_intra_process_comms(true)) {
  bus_    = this->declare_parameter<std::uint8_t>("bus", 1);
  addr_   = this->declare_parameter<std::uint8_t>("i2c_addr", 0x06);
  reg_    = this->declare_parameter<std::uint8_t>("angle_msb_reg", 0x03);
  bits_   = this->declare_parameter<std::uint16_t>("bits", 14);
  rate_   = this->declare_parameter<double>("rate_hz", 1.0);
  offset_ = this->declare_parameter<std::uint16_t>("offset", 0.0); 

  scale_ = 2.0 * M_PI / static_cast<double>(1u << bits_);

  dev_ = "/dev/i2c-" + std::to_string(bus_);
  fd_ = ::open(dev_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open " + dev_);
  }
  if (ioctl(fd_, I2C_SLAVE, static_cast<int>(addr_)) < 0) {
    ::close(fd_); fd_ = -1;
    throw std::runtime_error("I2C_SLAVE ioctl failed (addr=0x" + to_hex(addr_) + ")");
  }

  pub_ = this->create_publisher<Float64>("/arm/angle_rad", rclcpp::SensorDataQoS());

  const double s = (rate_ <= 0.0) ? 1.0 : (1.0 / rate_);
  set_sample_period_seconds(s);

  RCLCPP_INFO(get_logger(),
              "ArmAngleNode iniciado: dev=%s addr=0x%s reg=0x%02X bits=%d rate=%.3f Hz",
              dev_.c_str(), to_hex(addr_).c_str(), reg_, bits_, 1.0 / period_.count());
}

ArmAngleNode::~ArmAngleNode() {
  timer_.reset();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void ArmAngleNode::set_sample_period_seconds(double seconds) {
  if (seconds <= 0.0) seconds = 1.0;
  period_ = std::chrono::duration<double>(seconds);

  auto cb = [this]() { this->on_timer(); };
  timer_.reset();
  timer_ = this->create_wall_timer(period_, cb);
}

void ArmAngleNode::on_timer() {
  Float64 msg;
  const auto raw = read_raw_angle();
  if (raw.has_value()) {
    msg.data = (static_cast<double>(*raw) * scale_) - offset_;
    pub_->publish(msg);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "I2C read failed; skipping sample");
  }
}

std::optional<uint16_t> ArmAngleNode::read_raw_angle() {
  if (fd_ < 0) return std::nullopt;

  uint8_t reg = static_cast<std::uint8_t>(reg_);
  uint8_t buf[2] = {0, 0};
  i2c_msg msgs[2]{};

  msgs[0].addr  = static_cast<uint16_t>(addr_);
  msgs[0].flags = 0;
  msgs[0].len   = 1;
  msgs[0].buf   = &reg;

  msgs[1].addr  = static_cast<uint16_t>(addr_);
  msgs[1].flags = I2C_M_RD;
  msgs[1].len   = 2;
  msgs[1].buf   = buf;

  i2c_rdwr_ioctl_data xfer{};
  xfer.msgs  = msgs;
  xfer.nmsgs = 2;

  const int ret = ioctl(fd_, I2C_RDWR, &xfer);
  if (ret != 2) return std::nullopt;

  uint16_t raw = 0;
  if (bits_ == 14) {
    raw = static_cast<uint16_t>(((uint16_t)buf[0] << 6) | (buf[1] >> 2)) & 0x3FFF;
  } else if (bits_ == 12) {
    raw = static_cast<uint16_t>(((uint16_t)buf[0] << 4) | (buf[1] >> 4)) & 0x0FFF;
  } else {
    raw = static_cast<uint16_t>(((uint16_t)buf[0] << 8) | buf[1]);
  }
  return raw;
}

} // namespace sensors

// Default main function:
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sensors::ArmAngleNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    auto logger = rclcpp::get_logger("arm_angle_publisher");
    RCLCPP_FATAL(logger, "Failed to start ArmAngleNode: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

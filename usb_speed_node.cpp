#include "prop_arm_control/usb_speed_node.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <cstring>
#include <cstdlib>

using std_msgs::msg::Float64;

namespace prop_arm_control {

UsbSpeedNode::UsbSpeedNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("usb_speed_node", opts)
{
  dev_  = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
  baud_ = this->declare_parameter<int>("baud", 921600);
  bits_ = this->declare_parameter<int>("bits", 14);
  topic_= this->declare_parameter<std::string>("topic", "/prop_arm/motor_speed_est");

  pub_ = this->create_publisher<Float64>(topic_, rclcpp::SensorDataQoS());

  fd_ = ::open(dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ >= 0) {
    setup_serial(fd_, baud_);
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK); // bloqueante para simplificar
  }

  run_.store(true);
  reader_ = std::thread([this](){ this->reader_loop(); });
}

UsbSpeedNode::~UsbSpeedNode() {
  run_.store(false);
  if (reader_.joinable()) reader_.join();
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void UsbSpeedNode::reader_loop() {
  const double k = 2.0 * M_PI / static_cast<double>(1u << bits_);
  std::string buf;
  buf.reserve(1024);
  char tmp[256];

  while (rclcpp::ok() && run_.load()) {
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n <= 0) continue;
    buf.append(tmp, tmp + n);

    size_t pos;
    while ((pos = buf.find('\n')) != std::string::npos) {
      std::string line = buf.substr(0, pos);
      buf.erase(0, pos + 1);

      // expected: "<micros>,<raw>\r?\n"
      const char* s = line.c_str();
      char* endp = nullptr;
      uint64_t t_us = std::strtoull(s, &endp, 10);
      if (!endp || *endp != ',') continue;
      unsigned long raw = std::strtoul(endp + 1, nullptr, 10);

      const double theta = static_cast<double>(raw) * k;

      if (!have_prev_) {
        prev_theta_ = theta;
        prev_tus_ = t_us;
        have_prev_ = true;
        continue;
      }

      double dtheta = theta - prev_theta_;
      if (dtheta > M_PI)  dtheta -= 2.0 * M_PI;
      if (dtheta < -M_PI) dtheta += 2.0 * M_PI;

      uint64_t dt_us = (t_us >= prev_tus_) ? (t_us - prev_tus_) : 0;
      if (dt_us > 0) {
        double dt = static_cast<double>(dt_us) * 1e-6;
        Float64 out;
        out.data = dtheta / dt; // rad/s
        pub_->publish(out);
      }

      prev_theta_ = theta;
      prev_tus_ = t_us;
    }
  }
}

speed_t UsbSpeedNode::baud_to_flag(int baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
    default: return B921600;
  }
}

bool UsbSpeedNode::setup_serial(int fd, int baud) {
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) return false;

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  speed_t spd = baud_to_flag(baud);
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  tio.c_cc[VMIN]  = 1;
  tio.c_cc[VTIME] = 0;

  return (tcsetattr(fd, TCSANOW, &tio) == 0);
}

} // namespace prop_arm_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<prop_arm_control::UsbSpeedNode>());
  rclcpp::shutdown();
  return 0;
}

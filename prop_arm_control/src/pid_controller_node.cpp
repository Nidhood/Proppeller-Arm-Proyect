#include "prop_arm_control/pid_controller_node.hpp"

#include <cmath>
#include <cstdint>
#include <rclcpp/qos.hpp>

namespace
{
    constexpr double RAD2DEG = 57.29577951308232; // 180 / pi
} // namespace

PidControllerNode::PidControllerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("prop_arm_pid_controller", options)
{
    // Declare and read parameters
    declareAndGetParameters();

    // Compute discrete PID coefficients using Tustin approximation
    computeCoefficients();

    RCLCPP_INFO(
        get_logger(),
        "PID controller node started with: Kp=%.4f, Ki=%.4f, Kd=%.4f, Ts=%.4f s, "
        "ref=%.2f deg, PWM range [%.1f, %.1f] us",
        kp_, ki_, kd_, ts_, ref_deg_, pwm_min_us_, pwm_max_us_);

    // Publisher for PWM command (UInt16 in microseconds)
    pwm_pub_ = create_publisher<std_msgs::msg::UInt16>(pwm_topic_, 10);

    // Subscriber for angle feedback: SensorDataQoS (best-effort, volatile)
    angle_sub_ = create_subscription<std_msgs::msg::Float64>(
        angle_topic_,
        rclcpp::SensorDataQoS(), // best-effort QoS
        std::bind(&PidControllerNode::angleCallback, this, std::placeholders::_1));

    // Timer for the control loop
    const auto period = std::chrono::duration<double>(ts_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PidControllerNode::controlLoop, this));
}

// === Parameter declaration and reading ===
void PidControllerNode::declareAndGetParameters()
{
    // Gains and timing
    kp_ = this->declare_parameter<double>("pid.p", kp_);
    ki_ = this->declare_parameter<double>("pid.i", ki_);
    kd_ = this->declare_parameter<double>("pid.d", kd_);
    ts_ = this->declare_parameter<double>("Ts", ts_);

    // Reference angle in degrees
    ref_deg_ = this->declare_parameter<double>("ref_deg", ref_deg_);

    // PWM range
    pwm_min_us_ = this->declare_parameter<double>("pwm_min", pwm_min_us_);
    pwm_max_us_ = this->declare_parameter<double>("pwm_max", pwm_max_us_);

    // Topics
    angle_topic_ = this->declare_parameter<std::string>("angle_topic", "/arm_sim/angle_rad");
    pwm_topic_ = this->declare_parameter<std::string>("pwm_topic", "/arm_sim/esc/pwm_us");
}

// === Compute discrete PID coefficients (Tustin) ===
void PidControllerNode::computeCoefficients()
{
    if (ts_ <= 0.0)
    {
        RCLCPP_WARN(get_logger(), "Ts <= 0, forcing Ts = 0.01 s");
        ts_ = 0.01;
    }

    const double Ts = ts_;

    A_ = kp_ + (ki_ * Ts / 2.0) + (2.0 * kd_ / Ts);
    B_ = -kp_ + (ki_ * Ts / 2.0) - (4.0 * kd_ / Ts);
    C_ = 2.0 * kd_ / Ts;

    RCLCPP_INFO(
        get_logger(),
        "Discrete PID (Tustin) coefficients: A=%.6f, B=%.6f, C=%.6f",
        A_, B_, C_);
}

// === Angle feedback callback ===
void PidControllerNode::angleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Incoming angle is in radians
    last_angle_rad_ = msg->data;
    last_angle_deg_ = last_angle_rad_ * RAD2DEG;
    angle_received_ = true;
}

// === Main control loop (runs every Ts seconds) ===
void PidControllerNode::controlLoop()
{
    if (!angle_received_)
    {
        // No feedback yet, do not run PID
        return;
    }

    // Error update
    e_[2] = e_[1];
    e_[1] = e_[0];
    e_[0] = ref_deg_ - last_angle_deg_;

    // Incremental PID (Tustin)
    const double du = A_ * e_[0] + B_ * e_[1] + C_ * e_[2];

    u_[0] = u_[1] + du;

    // Saturate PWM in double domain
    if (u_[0] > pwm_max_us_)
    {
        u_[0] = pwm_max_us_;
    }
    else if (u_[0] < pwm_min_us_)
    {
        u_[0] = pwm_min_us_;
    }

    // Publish command as UInt16 PWM (microseconds)
    const auto pwm_cmd_us = static_cast<std::uint16_t>(std::lround(u_[0]));
    std_msgs::msg::UInt16 msg;
    msg.data = pwm_cmd_us;
    pwm_pub_->publish(msg);

    // Prepare for next step
    u_[1] = u_[0];
}

// === main() ===
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

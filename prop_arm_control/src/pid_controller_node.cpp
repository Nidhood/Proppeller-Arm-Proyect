#include "prop_arm_control/pid_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include <rclcpp/qos.hpp>

namespace {
constexpr double RAD2DEG = 57.29577951308232; // 180 / pi
constexpr double DEG2RAD = 1.0 / RAD2DEG;
} // namespace

using std::placeholders::_1;

PidControllerNode::PidControllerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("prop_arm_pid_controller", options) {
    // Declare and read parameters
    declareAndGetParameters();

    // Initial gain scheduling and discrete coefficients
    updateScheduledGains();

    // Publisher for PWM command (UInt16 microseconds)
    pwm_pub_ = create_publisher<std_msgs::msg::UInt16>(pwm_topic_, 10);

    // Subscriber for angle feedback (sensor QoS)
    angle_sub_ = create_subscription<std_msgs::msg::Float64>(
                     angle_topic_, rclcpp::SensorDataQoS(),
                     std::bind(&PidControllerNode::angleCallback, this, _1));

    // Subscriber for reference angle in radians
    ref_sub_ = create_subscription<std_msgs::msg::Float64>(
                   ref_angle_topic_, rclcpp::QoS(10).best_effort(),
                   std::bind(&PidControllerNode::refCallback, this, _1));

    // Subscriber for auto/manual mode
    auto_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
                         auto_mode_topic_, rclcpp::QoS(10).best_effort(),
                         std::bind(&PidControllerNode::autoModeCallback, this, _1));

    // Timer for control loop
    const auto period = std::chrono::duration<double>(ts_);
    timer_ = create_wall_timer(
                 std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                 std::bind(&PidControllerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "PID controller with gravity feedforward, scheduled gains and "
                "auto_mode initialized. "
                "Ts=%.4f s, pwm_idle=%.2f, kff=%.2f, auto_mode=%s",
                ts_, pwm_idle_us_, kff_pwm_per_sin_,
                auto_mode_enabled_ ? "true" : "false");
}

void PidControllerNode::declareAndGetParameters() {
    // Sampling time
    ts_ = this->declare_parameter<double>("Ts", ts_);
    if (ts_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "Ts <= 0.0, forcing Ts = 0.01 s");
        ts_ = 0.01;
    }

    // Base PID gains (fallback)
    kp_ = this->declare_parameter<double>("pid.p", 0.10);
    ki_ = this->declare_parameter<double>("pid.i", 0.80);
    kd_ = this->declare_parameter<double>("pid.d", 0.08);
    i_clamp_ = this->declare_parameter<double>("pid.i_clamp", i_clamp_);

    // Low / mid / high gains (if not set, default to base gains)
    kp_low_ = this->declare_parameter<double>("pid_low.p", kp_);
    ki_low_ = this->declare_parameter<double>("pid_low.i", ki_);
    kd_low_ = this->declare_parameter<double>("pid_low.d", kd_);

    kp_mid_ = this->declare_parameter<double>("pid_mid.p", kp_);
    ki_mid_ = this->declare_parameter<double>("pid_mid.i", ki_);
    kd_mid_ = this->declare_parameter<double>("pid_mid.d", kd_);

    kp_high_ = this->declare_parameter<double>("pid_high.p", kp_);
    ki_high_ = this->declare_parameter<double>("pid_high.i", ki_);
    kd_high_ = this->declare_parameter<double>("pid_high.d", kd_);

    // Reference in degrees (fallback if no topic is used)
    ref_deg_ = this->declare_parameter<double>("ref_deg", ref_deg_);

    // Scheduling centers
    angle_low_center_deg_ = this->declare_parameter<double>(
                                "angle_low_center_deg", angle_low_center_deg_);
    angle_mid_center_deg_ = this->declare_parameter<double>(
                                "angle_mid_center_deg", angle_mid_center_deg_);
    angle_high_center_deg_ = this->declare_parameter<double>(
                                 "angle_high_center_deg", angle_high_center_deg_);

    // PWM range
    pwm_min_us_ = this->declare_parameter<double>("pwm_min", pwm_min_us_);
    pwm_max_us_ = this->declare_parameter<double>("pwm_max", pwm_max_us_);

    // Feedforward parameters
    pwm_idle_us_ = this->declare_parameter<double>("pwm_idle_us", pwm_idle_us_);
    kff_pwm_per_sin_ =
        this->declare_parameter<double>("kff_pwm_per_sin", kff_pwm_per_sin_);
    ff_enabled_ = this->declare_parameter<bool>("enable_ff", ff_enabled_);

    // Auto mode initial state (true = control enabled)
    auto_mode_enabled_ =
        this->declare_parameter<bool>("auto_mode_enabled", auto_mode_enabled_);

    // Topics
    angle_topic_ =
        this->declare_parameter<std::string>("angle_topic", "/arm_sim/angle_rad");
    pwm_topic_ =
        this->declare_parameter<std::string>("pwm_topic", "/arm_sim/esc/pwm_us");
    ref_angle_topic_ = this->declare_parameter<std::string>(
                           "ref_angle_topic", "/arm_sim/ref_angle_rad");
    auto_mode_topic_ = this->declare_parameter<std::string>(
                           "auto_mode_topic", "/arm_sim/esc/auto_mode");

    RCLCPP_INFO(get_logger(),
                "Params: base PID (p=%.4f, i=%.4f, d=%.4f), "
                "low (p=%.4f, i=%.4f, d=%.4f), mid (p=%.4f, i=%.4f, d=%.4f), "
                "high (p=%.4f, i=%.4f, d=%.4f)",
                kp_, ki_, kd_, kp_low_, ki_low_, kd_low_, kp_mid_, ki_mid_,
                kd_mid_, kp_high_, ki_high_, kd_high_);
}

void PidControllerNode::updateScheduledGains() {
    // Use reference angle in degrees for scheduling
    const double ref = ref_deg_;

    // Assume low < mid < high
    const double c_low = angle_low_center_deg_;
    const double c_mid = angle_mid_center_deg_;
    const double c_high = angle_high_center_deg_;

    if (ref <= c_low) {
        kp_ = kp_low_;
        ki_ = ki_low_;
        kd_ = kd_low_;
    } else if (ref >= c_high) {
        kp_ = kp_high_;
        ki_ = ki_high_;
        kd_ = kd_high_;
    } else if (ref <= c_mid) {
        const double alpha = (ref - c_low) / (c_mid - c_low);
        kp_ = (1.0 - alpha) * kp_low_ + alpha * kp_mid_;
        ki_ = (1.0 - alpha) * ki_low_ + alpha * ki_mid_;
        kd_ = (1.0 - alpha) * kd_low_ + alpha * kd_mid_;
    } else {
        const double alpha = (ref - c_mid) / (c_high - c_mid);
        kp_ = (1.0 - alpha) * kp_mid_ + alpha * kp_high_;
        ki_ = (1.0 - alpha) * ki_mid_ + alpha * ki_high_;
        kd_ = (1.0 - alpha) * kd_mid_ + alpha * kd_high_;
    }

    // Once gains are updated, recompute discrete coefficients
    computeCoefficients();
}

void PidControllerNode::computeCoefficients() {
    const double Ts = ts_;

    if (Ts <= 0.0) {
        RCLCPP_WARN(get_logger(),
                    "Ts <= 0 in computeCoefficients, using Ts = 0.01 s internally");
    }

    const double Ts_eff = (Ts > 0.0) ? Ts : 0.01;

    // Tustin-based incremental PID coefficients
    A_ = kp_ + (ki_ * Ts_eff / 2.0) + (2.0 * kd_ / Ts_eff);
    B_ = -kp_ + (ki_ * Ts_eff / 2.0) - (4.0 * kd_ / Ts_eff);
    C_ = 2.0 * kd_ / Ts_eff;

    RCLCPP_DEBUG(get_logger(),
                 "Updated discrete PID (Tustin): kp=%.5f, ki=%.5f, kd=%.5f | "
                 "A=%.6f, B=%.6f, C=%.6f",
                 kp_, ki_, kd_, A_, B_, C_);
}

double
PidControllerNode::computeGravityFeedforward(double ref_deg) const noexcept {
    if (!ff_enabled_) {
        return pwm_idle_us_;
    }

    const double ref_rad = ref_deg * DEG2RAD;
    const double s = std::sin(ref_rad);

    double u_ff = pwm_idle_us_ + kff_pwm_per_sin_ * s;

    // No clamp here, clamp is applied to final u_cmd
    return u_ff;
}

void PidControllerNode::angleCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
    last_angle_rad_ = msg->data;
    last_angle_deg_ = last_angle_rad_ * RAD2DEG;
    angle_received_ = true;
}

void PidControllerNode::refCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
    const double ref_rad = msg->data;
    ref_deg_ = ref_rad * RAD2DEG;
    ref_received_ = true;
}

void PidControllerNode::autoModeCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    auto_mode_enabled_ = msg->data;
    RCLCPP_INFO(get_logger(), "auto_mode changed: %s",
                auto_mode_enabled_ ? "ENABLED (automatic control)"
                : "DISABLED (manual / no output)");
}

void PidControllerNode::controlLoop() {
    if (!angle_received_) {
        return;
    }

    // If auto mode is disabled, do not compute or publish control
    if (!auto_mode_enabled_) {
        return;
    }

    // Update scheduled gains based on current reference
    updateScheduledGains();

    // Error update: e[k], e[k-1], e[k-2]
    e_[2] = e_[1];
    e_[1] = e_[0];
    e_[0] = ref_deg_ - last_angle_deg_;

    // Incremental PID (Tustin)
    const double du = A_ * e_[0] + B_ * e_[1] + C_ * e_[2];
    u_pid_[0] = u_pid_[1] + du;

    // Clamp incremental PID contribution to avoid integral wind-up
    if (u_pid_[0] > i_clamp_)
        u_pid_[0] = i_clamp_;
    if (u_pid_[0] < -i_clamp_)
        u_pid_[0] = -i_clamp_;

    // Gravity feedforward based on reference
    const double u_ff = computeGravityFeedforward(ref_deg_);

    // Total PWM command = feedforward + PID contribution
    double u_cmd = u_ff + u_pid_[0];

    // Saturate final PWM
    if (u_cmd > pwm_max_us_) {
        u_cmd = pwm_max_us_;
    } else if (u_cmd < pwm_min_us_) {
        u_cmd = pwm_min_us_;
    }

    // Publish as UInt16
    const auto pwm_cmd_us = static_cast<std::uint16_t>(std::lround(u_cmd));
    std_msgs::msg::UInt16 msg;
    msg.data = pwm_cmd_us;
    pwm_pub_->publish(msg);

    // Prepare for next step
    u_pid_[1] = u_pid_[0];
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
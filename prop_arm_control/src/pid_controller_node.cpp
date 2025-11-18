#include "prop_arm_control/pid_controller_node.hpp"

#include <cmath>
#include <cstdint>
#include <rclcpp/qos.hpp>

namespace
{
    constexpr double RAD2DEG = 57.29577951308232; // 180 / pi
    constexpr double DEG2RAD = 1.0 / RAD2DEG;
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
        "PID node started: Kp=%.4f Ki=%.4f Kd=%.4f Ts=%.4f, ref_deg=%.2f, "
        "pwm_min=%.1f pwm_max=%.1f, pwm_idle=%.1f, kff=%.1f, ff=%s",
        kp_, ki_, kd_, ts_, ref_deg_,
        pwm_min_us_, pwm_max_us_,
        pwm_idle_us_, kff_pwm_per_sin_,
        ff_enabled_ ? "true" : "false");

    // Publisher for PWM command (UInt16 in microseconds)
    pwm_pub_ = create_publisher<std_msgs::msg::UInt16>(pwm_topic_, 10);

    // Subscriber for angle feedback (radians)
    angle_sub_ = create_subscription<std_msgs::msg::Float64>(
        angle_topic_,
        rclcpp::SensorDataQoS(), // best-effort QoS
        std::bind(&PidControllerNode::angleCallback, this, std::placeholders::_1));

    // Subscriber for reference angle in radians
    ref_sub_ = create_subscription<std_msgs::msg::Float64>(
        ref_angle_topic_,
        rclcpp::QoS(10).best_effort(),
        std::bind(&PidControllerNode::refCallback, this, std::placeholders::_1));

    // Timer for the control loop
    const auto period = std::chrono::duration<double>(ts_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PidControllerNode::controlLoop, this));
}

// Parameter declaration and reading.
void PidControllerNode::declareAndGetParameters()
{
    // Gains and timing
    kp_ = this->declare_parameter<double>("pid.p", kp_);
    ki_ = this->declare_parameter<double>("pid.i", ki_);
    kd_ = this->declare_parameter<double>("pid.d", kd_);
    ts_ = this->declare_parameter<double>("Ts", ts_);

    // Initial reference angle in degrees
    ref_deg_ = this->declare_parameter<double>("ref_deg", ref_deg_);

    // PWM range
    pwm_min_us_ = this->declare_parameter<double>("pwm_min", pwm_min_us_);
    pwm_max_us_ = this->declare_parameter<double>("pwm_max", pwm_max_us_);

    // Feedforward parameters
    pwm_idle_us_ = this->declare_parameter<double>("pwm_idle_us", pwm_idle_us_);
    kff_pwm_per_sin_ =
        this->declare_parameter<double>("kff_pwm_per_sin", kff_pwm_per_sin_);
    ff_enabled_ = this->declare_parameter<bool>("enable_ff", ff_enabled_);

    // Topics
    angle_topic_ =
        this->declare_parameter<std::string>("angle_topic", "/arm_sim/angle_rad");
    pwm_topic_ =
        this->declare_parameter<std::string>("pwm_topic", "/arm_sim/esc/pwm_us");
    ref_angle_topic_ = this->declare_parameter<std::string>(
        "ref_angle_topic", "/arm_sim/ref_angle_rad");
}

// Compute discrete PID coefficients (Tustin incremental form).
void PidControllerNode::computeCoefficients()
{
    if (ts_ <= 0.0)
    {
        RCLCPP_WARN(get_logger(), "Ts <= 0, forcing Ts = 0.01 s");
        ts_ = 0.01;
    }

    const double Ts = ts_;

    // Incremental (velocity) form coefficients with Tustin:
    // du[k] = A*e[k] + B*e[k-1] + C*e[k-2]
    A_ = kp_ + (ki_ * Ts / 2.0) + (2.0 * kd_ / Ts);
    B_ = -kp_ + (ki_ * Ts / 2.0) - (4.0 * kd_ / Ts);
    C_ = 2.0 * kd_ / Ts;

    RCLCPP_INFO(
        get_logger(),
        "Discrete PID (Tustin incremental) coefficients: A=%.6f, B=%.6f, C=%.6f",
        A_, B_, C_);
}

// Angle feedback callback (radians).
void PidControllerNode::angleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    last_angle_rad_ = msg->data;
    last_angle_deg_ = last_angle_rad_ * RAD2DEG;
    angle_received_ = true;
}

// Reference callback: ref angle in radians on ref_angle_topic_.
void PidControllerNode::refCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    double ref_rad = msg->data;
    double ref_deg = ref_rad * RAD2DEG;

    // Limit reference to [0, 90] deg for safety
    if (ref_deg < 0.0)
        ref_deg = 0.0;
    if (ref_deg > 90.0)
        ref_deg = 90.0;

    ref_deg_ = ref_deg;
}

// Gravity feedforward based on reference angle (degrees).
double PidControllerNode::computeGravityFeedforward(double ref_deg) const noexcept
{
    // If feedforward disabled, use idle PWM as a simple center point
    if (!ff_enabled_)
    {
        return pwm_idle_us_;
    }

    double ref_rad = ref_deg * DEG2RAD;
    double pwm_eq = pwm_idle_us_ + kff_pwm_per_sin_ * std::sin(ref_rad);

    return pwm_eq;
}

// Main control loop (runs every Ts seconds).
void PidControllerNode::controlLoop()
{
    if (!angle_received_)
    {
        // No feedback yet, do not run PID
        return;
    }

    // 1) Update error history
    e_[2] = e_[1];
    e_[1] = e_[0];
    e_[0] = ref_deg_ - last_angle_deg_;

    // 2) Incremental PID (velocity form with Tustin)
    const double du = A_ * e_[0] + B_ * e_[1] + C_ * e_[2];

    // Update PID output (this is the incremental part around 0)
    u_pid_[0] = u_pid_[1] + du;

    // (Optional) You could clamp u_pid_[] here if you want a PID-only limit
    // double u_pid_min = -500.0;
    // double u_pid_max =  500.0;
    // if (u_pid_[0] < u_pid_min) u_pid_[0] = u_pid_min;
    // if (u_pid_[0] > u_pid_max) u_pid_[0] = u_pid_max;

    // 3) Gravity feedforward from reference angle
    const double u_ff = computeGravityFeedforward(ref_deg_);

    // 4) Total command in microseconds
    double u_cmd = u_ff + u_pid_[0];

    // 5) Saturate final PWM command
    if (u_cmd > pwm_max_us_)
    {
        u_cmd = pwm_max_us_;
    }
    else if (u_cmd < pwm_min_us_)
    {
        u_cmd = pwm_min_us_;
    }

    // 6) Publish command as UInt16 PWM (microseconds)
    const auto pwm_cmd_us = static_cast<std::uint16_t>(std::lround(u_cmd));
    std_msgs::msg::UInt16 msg;
    msg.data = pwm_cmd_us;
    pwm_pub_->publish(msg);

    // 7) Prepare for next step
    u_pid_[1] = u_pid_[0];
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

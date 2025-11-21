#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>

class PidControllerNode : public rclcpp::Node {
public:
    explicit PidControllerNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_mode_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Sampling time [s]
    double ts_{0.01};

    // Effective PID gains (after scheduling)
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};

    // Low / mid / high PID gains
    double kp_low_{0.0}, ki_low_{0.0}, kd_low_{0.0};
    double kp_mid_{0.0}, ki_mid_{0.0}, kd_mid_{0.0};
    double kp_high_{0.0}, ki_high_{0.0}, kd_high_{0.0};

    // Scheduling centers (in degrees)
    double angle_low_center_deg_{15.0};
    double angle_mid_center_deg_{45.0};
    double angle_high_center_deg_{75.0};

    // Feedforward parameters
    double pwm_idle_us_{1182.0};
    double kff_pwm_per_sin_{176.0};
    bool ff_enabled_{true};

    // Reference angle [deg]
    double ref_deg_{0.0};

    // PWM range [us]
    double pwm_min_us_{1000.0};
    double pwm_max_us_{2000.0};

    // Integral clamp for PID contribution [us]
    double i_clamp_{100.0};

    // Discrete PID coefficients (Tustin)
    double A_{0.0};
    double B_{0.0};
    double C_{0.0};

    // PID internal state:
    // e[0] = e[k], e[1] = e[k-1], e[2] = e[k-2]
    std::array<double, 3> e_{{0.0, 0.0, 0.0}};

    // u_pid[0] = u_pid[k], u_pid[1] = u_pid[k-1] (incremental PID contribution)
    std::array<double, 2> u_pid_{{0.0, 0.0}};

    // Feedback state
    double last_angle_rad_{0.0};
    double last_angle_deg_{0.0};
    bool angle_received_{false};
    bool ref_received_{false};

    // Auto/manual mode
    bool auto_mode_enabled_{true};

    // Topic names
    std::string angle_topic_;
    std::string pwm_topic_;
    std::string ref_angle_topic_;
    std::string auto_mode_topic_;

    // Parameter helpers
    void declareAndGetParameters();
    void updateScheduledGains();
    void computeCoefficients();
    double computeGravityFeedforward(double ref_deg) const noexcept;

    // Callbacks
    void angleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void refCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void autoModeCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void controlLoop();
};
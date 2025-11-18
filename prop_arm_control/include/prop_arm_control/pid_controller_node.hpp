#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>

class PidControllerNode : public rclcpp::Node
{
public:
    explicit PidControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:

    // ROS handlers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PID parameters
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double ts_{1.0};
    double ref_deg_{0.0};

    // PWM range [us]
    double pwm_min_us_{1000.0};
    double pwm_max_us_{2000.0};

    // Discrete PID coefficients (Tustin)
    double A_{0.0};
    double B_{0.0};
    double C_{0.0};

    // PID state: e[0] = e[k], e[1] = e[k-1], e[2] = e[k-2]
    std::array<double, 3> e_{{0.0, 0.0, 0.0}};
    // u[0] = u[k], u[1] = u[k-1] (PWM command in microseconds)
    std::array<double, 2> u_{{1500.0, 1500.0}};

    // Feedback state
    double last_angle_rad_{0.0};
    double last_angle_deg_{0.0};
    bool angle_received_{false};

    // Topic names
    std::string angle_topic_;
    std::string pwm_topic_;

    // Internal helpers
    void angleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void controlLoop();
    void computeCoefficients();
    void declareAndGetParameters();
};

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <string>

namespace prop_arm_ctrl
{

    class AngleHoldSSController : public rclcpp::Node
    {
    public:
        explicit AngleHoldSSController(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

    private:
        // Build plant + precompensator
        void computeKr();
        // Periodic control
        void loop();

        // --- Params (minimal) ---
        double rate_hz_{250.0};
        double theta_ref_deg_{0.0};
        double u_min_{0.0}, u_max_{785.0};
        double omega0_rad_s_{400.0};

        // Plant constants (adapted to your sim)
        double Ja_{4.5e-4};
        double Ke_{5.0e-4};
        double La_{0.15};
        double Kt_lin_{0.5};
        double tau_{0.01};       // motor time constant (s)
        double deriv_tau_{0.01}; // LPF for d/dt

        // State-space: x=[theta, omega, Δv_emf], y=theta
        Eigen::Matrix3d A_{Eigen::Matrix3d::Zero()};
        Eigen::Vector3d B_{Eigen::Vector3d::Zero()};
        Eigen::RowVector3d C_{Eigen::RowVector3d::Zero()};
        Eigen::RowVector3d K_{Eigen::RowVector3d::Zero()};
        double Kr_{0.0};

        // I/O
        std::string topic_angle_deg_{"/prop_arm/arm_angle_deg"};
        std::string topic_delta_vemf_{"/prop_arm/delta_v_emf"};
        std::string topic_cmd_vel_{"/velocity_controller/commands"};

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_angle_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_delta_vemf_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
        rclcpp::TimerBase::SharedPtr timer_;

        // State from topics + estimation
        bool have_angle_{false}, have_vemf_{false};
        double theta_deg_{0.0};
        double delta_vemf_{0.0};

        bool have_prev_{false};
        double theta_prev_{0.0};
        rclcpp::Time t_prev_;
        double omega_est_{0.0};
    };

} // namespace prop_arm_ctrl

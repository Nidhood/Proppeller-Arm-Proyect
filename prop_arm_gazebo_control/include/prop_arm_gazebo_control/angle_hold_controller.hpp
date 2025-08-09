#pragma once
// Angle hold state-feedback controller using velocity commands
// u = Kr * r - K * x , with Kr = - ( C * (A - B*K)^(-1) * B )^(-1)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>

namespace prop_arm_ctrl
{
    class AngleHoldController : public rclcpp::Node
    {
    public:
        explicit AngleHoldController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        // === ROS callbacks / loop ===
        void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
        void controlLoop();
        void refreshSetpointFromParam();

        // === Model / control matrices ===
        Eigen::Matrix3d A_;    // 3x3
        Eigen::Vector3d B_;    // 3x1
        Eigen::RowVector3d C_; // 1x3  (y = theta)
        Eigen::RowVector3d K_; // 1x3
        double Kr_{1.0};       // scalar precompensator

        // === State & refs ===
        double theta_{0.0};   // Current arm angle (radians)
        double omega_{0.0};   // Current arm angular velocity (rad/s)
        double v_emf_{0.0};   // Motor back EMF estimate
        double ref_rad_{0.0}; // Reference angle (radians)
        bool have_state_{false};

        // === Control limits ===
        double u_min_{-785.0}; // Min motor velocity (rad/s)
        double u_max_{785.0};  // Max motor velocity (rad/s)

        // === ROS I/O ===
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
        rclcpp::TimerBase::SharedPtr loop_timer_;

        // === Configuration ===
        std::string joint_name_{"arm_link_joint"};
        std::string velocity_topic_{"/velocity_controller/commands"};
        double loop_rate_hz_{250.0};

        // === Model parameters ===
        double Ja_{0.002}; // Arm inertia
        double Jm_{0.001}; // Motor inertia
        double Kt_{0.02};  // Motor torque constant
        double La_{0.001}; // Inductance
        double Ke_{0.02};  // Back EMF constant
        double Km_{1.0};   // Motor constant
        double Rm_{2.0};   // Motor resistance
        double Rs_{0.5};   // Series resistance
        double Kf_{0.1};   // Friction coefficient

        // === Helpers ===
        void computeKr();              // Recompute Kr after K/A/B/C changes
        void buildPlantFromDescriptor( // Convert E xdot = A0 x + B0 u to xdot = A x + B u
            const Eigen::Matrix3d &E, const Eigen::Matrix3d &A0, const Eigen::Vector3d &B0);
        void initializeControlSystem(); // Initialize the complete control system
    };

} // namespace prop_arm_ctrl
#include "prop_arm_gazebo_control/angle_hold_controller.hpp"
#include <algorithm>
#include <vector>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::RowVector3d;
using Eigen::Vector3d;

namespace prop_arm_ctrl
{

    AngleHoldController::AngleHoldController(const rclcpp::NodeOptions &options)
        : rclcpp::Node("angle_hold_controller", options)
    {
        // -------- Declare parameters (editable at runtime) --------
        this->declare_parameter<std::vector<double>>("K", {15.0, 8.0, 2.0}); // More aggressive gains
        this->declare_parameter<double>("theta_ref_deg", 45.0);
        this->declare_parameter<double>("u_min", -785.0);
        this->declare_parameter<double>("u_max", 785.0);
        this->declare_parameter<double>("rate_hz", 250.0);
        this->declare_parameter<std::string>("joint_name", joint_name_);
        this->declare_parameter<std::string>("velocity_topic", velocity_topic_);

        // Model parameters (can be tuned)
        this->declare_parameter<double>("Ja", Ja_);
        this->declare_parameter<double>("Jm", Jm_);
        this->declare_parameter<double>("Kt", Kt_);
        this->declare_parameter<double>("La", La_);
        this->declare_parameter<double>("Ke", Ke_);
        this->declare_parameter<double>("Km", Km_);
        this->declare_parameter<double>("Rm", Rm_);
        this->declare_parameter<double>("Rs", Rs_);
        this->declare_parameter<double>("Kf", Kf_);

        // Read parameters
        this->get_parameter("Ja", Ja_);
        this->get_parameter("Jm", Jm_);
        this->get_parameter("Kt", Kt_);
        this->get_parameter("La", La_);
        this->get_parameter("Ke", Ke_);
        this->get_parameter("Km", Km_);
        this->get_parameter("Rm", Rm_);
        this->get_parameter("Rs", Rs_);
        this->get_parameter("Kf", Kf_);

        u_min_ = this->get_parameter("u_min").as_double();
        u_max_ = this->get_parameter("u_max").as_double();
        loop_rate_hz_ = this->get_parameter("rate_hz").as_double();
        this->get_parameter("joint_name", joint_name_);
        this->get_parameter("velocity_topic", velocity_topic_);

        // Initialize control system matrices
        initializeControlSystem();

        // -------- ROS I/O --------
        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&AngleHoldController::jointStateCb, this, std::placeholders::_1));

        // IMPORTANT: Use velocity controller instead of effort controller
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            velocity_topic_, 10);

        loop_timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(1e6 / loop_rate_hz_)),
            std::bind(&AngleHoldController::controlLoop, this));

        RCLCPP_INFO(get_logger(),
                    "AngleHoldController started:");
        RCLCPP_INFO(get_logger(),
                    "  Joint: '%s'", joint_name_.c_str());
        RCLCPP_INFO(get_logger(),
                    "  Velocity topic: '%s'", velocity_topic_.c_str());
        RCLCPP_INFO(get_logger(),
                    "  Control rate: %.1f Hz", loop_rate_hz_);
        RCLCPP_INFO(get_logger(),
                    "  Precompensator Kr: %.4f", Kr_);
        RCLCPP_INFO(get_logger(),
                    "  State feedback K: [%.3f %.3f %.3f]", K_(0, 0), K_(0, 1), K_(0, 2));
        RCLCPP_INFO(get_logger(),
                    "  Velocity limits: [%.1f, %.1f] rad/s", u_min_, u_max_);
    }

    void AngleHoldController::initializeControlSystem()
    {
        // -------- Build plant from descriptor form E xdot = A0 x + B0 u --------
        // States: x = [theta, omega, v_emf]
        Matrix3d E = Matrix3d::Identity();
        E(1, 1) = Ja_; // Arm inertia
        E(2, 2) = Jm_; // Motor inertia

        Matrix3d A0 = Matrix3d::Zero();
        A0(0, 1) = 1.0;                              // theta_dot = omega
        A0(1, 2) = (Kt_ * La_) / Ke_;                // Arm dynamics coupling
        A0(2, 2) = -(Km_ * Ke_) / (Rm_ + Rs_) + Kf_; // Motor back EMF dynamics

        Vector3d B0 = Vector3d::Zero();
        B0(2) = (Km_ * Ke_) / (Rm_ + Rs_); // Motor input coupling

        // Output matrix: y = theta (we want to control arm angle)
        C_.resize(1, 3);
        C_ << 1.0, 0.0, 0.0;

        // Convert descriptor form to standard form
        buildPlantFromDescriptor(E, A0, B0);

        // -------- Set feedback gains K --------
        std::vector<double> kvec;
        this->get_parameter("K", kvec);
        if (kvec.size() != 3)
        {
            RCLCPP_WARN(get_logger(), "Invalid K vector size, using defaults");
            kvec = {15.0, 8.0, 2.0};
        }
        K_.resize(1, 3);
        K_ << kvec[0], kvec[1], kvec[2];

        // Compute precompensator for zero steady-state error
        computeKr();

        // Get initial setpoint
        refreshSetpointFromParam();

        RCLCPP_INFO(get_logger(), "Control system initialized successfully");
    }

    void AngleHoldController::buildPlantFromDescriptor(const Matrix3d &E, const Matrix3d &A0, const Vector3d &B0)
    {
        // Convert descriptor form E*xdot = A0*x + B0*u to standard form xdot = A*x + B*u
        A_ = E.lu().solve(A0);
        B_ = E.lu().solve(B0);

        RCLCPP_DEBUG(get_logger(), "Plant matrices computed:");
        RCLCPP_DEBUG(get_logger(), "A matrix:");
        for (int i = 0; i < 3; ++i)
        {
            RCLCPP_DEBUG(get_logger(), "  [%.6f %.6f %.6f]", A_(i, 0), A_(i, 1), A_(i, 2));
        }
        RCLCPP_DEBUG(get_logger(), "B vector: [%.6f %.6f %.6f]", B_(0), B_(1), B_(2));
    }

    void AngleHoldController::computeKr()
    {
        // Compute precompensator: Kr = -1 / ( C * (A - B*K)^(-1) * B )
        try
        {
            Eigen::Matrix3d A_BK = A_ - B_ * K_;
            Eigen::Vector3d invTerm = A_BK.lu().solve(B_);
            double denominator = (C_ * invTerm)(0, 0);

            if (std::abs(denominator) < 1e-10)
            {
                RCLCPP_WARN(get_logger(), "Small denominator in Kr computation, using Kr=1.0");
                Kr_ = 1.0;
            }
            else
            {
                Kr_ = -1.0 / denominator;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Error computing Kr: %s, using Kr=1.0", e.what());
            Kr_ = 1.0;
        }

        RCLCPP_DEBUG(get_logger(), "Precompensator Kr computed: %.6f", Kr_);
    }

    void AngleHoldController::refreshSetpointFromParam()
    {
        double deg = this->get_parameter("theta_ref_deg").as_double();
        ref_rad_ = deg * M_PI / 180.0;
        RCLCPP_DEBUG(get_logger(), "Reference angle updated: %.1f° (%.4f rad)", deg, ref_rad_);
    }

    // ---- Callbacks / main control loop ----
    void AngleHoldController::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == joint_name_)
            {
                if (i < msg->position.size())
                {
                    theta_ = msg->position[i];
                }
                if (i < msg->velocity.size())
                {
                    omega_ = msg->velocity[i];
                }
                have_state_ = true;
                break;
            }
        }
    }

    void AngleHoldController::controlLoop()
    {
        // Refresh setpoint in case parameter was changed
        refreshSetpointFromParam();

        if (!have_state_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "No joint state received yet, skipping control");
            return;
        }

        // State vector: x = [theta, omega, v_emf]
        // Note: v_emf is not directly measured, so we estimate it or set to 0
        Eigen::Vector3d x;
        x << theta_, omega_, v_emf_; // Using previous v_emf estimate or 0

        // State feedback control law: u = Kr * r - K * x
        double u_feedback = Kr_ * ref_rad_ - (K_ * x)(0, 0);

        // Apply velocity limits (motor speed limits)
        double u_cmd = std::clamp(u_feedback, u_min_, u_max_);

        // Publish velocity command
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {u_cmd};
        velocity_pub_->publish(cmd);

        // Log control action periodically
        static int log_counter = 0;
        if (++log_counter >= 250)
        { // Log every 1 second at 250Hz
            log_counter = 0;
            double error_deg = (ref_rad_ - theta_) * 180.0 / M_PI;
            RCLCPP_INFO(get_logger(),
                        "Control: ref=%.1f°, actual=%.1f°, error=%.1f°, cmd=%.1f rad/s",
                        ref_rad_ * 180.0 / M_PI, theta_ * 180.0 / M_PI,
                        error_deg, u_cmd);
        }

        // Simple v_emf estimation for next iteration (optional)
        // In practice, you might estimate this from motor model or set to 0
        v_emf_ = 0.0; // Simplified: assume zero for now
    }

} // namespace prop_arm_ctrl

// -------- Node main --------
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(prop_arm_ctrl::AngleHoldController)

#include "prop_arm_gazebo_control/angle_hold_ss_controller.hpp"
#include <algorithm>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::RowVector3d;
using Eigen::Vector3d;

namespace prop_arm_ctrl
{

    AngleHoldSSController::AngleHoldSSController(const rclcpp::NodeOptions &opts)
        : rclcpp::Node("angle_hold_ss_controller", opts)
    {
        // --- Params ---
        declare_parameter("rate_hz", rate_hz_);
        declare_parameter("theta_ref_deg", theta_ref_deg_);
        declare_parameter("u_min", u_min_);
        declare_parameter("u_max", u_max_);

        declare_parameter("Ja", Ja_);
        declare_parameter("Ke", Ke_);
        declare_parameter("La", La_);
        declare_parameter("Kt_lin", Kt_lin_);
        declare_parameter("tau", tau_);
        declare_parameter("deriv_tau", deriv_tau_);
        declare_parameter("omega0_rad_s", omega0_rad_s_);

        std::vector<double> Kdef{15.0, 8.0, 2.0}; // K=[K1 K2 K3]
        declare_parameter<std::vector<double>>("K", Kdef);

        declare_parameter("topic_angle_deg", topic_angle_deg_);
        declare_parameter("topic_delta_vemf", topic_delta_vemf_);
        declare_parameter("topic_cmd_vel", topic_cmd_vel_);

        // Load
        rate_hz_ = get_parameter("rate_hz").as_double();
        theta_ref_deg_ = get_parameter("theta_ref_deg").as_double();
        u_min_ = get_parameter("u_min").as_double();
        u_max_ = get_parameter("u_max").as_double();
        Ja_ = get_parameter("Ja").as_double();
        Ke_ = get_parameter("Ke").as_double();
        La_ = get_parameter("La").as_double();
        Kt_lin_ = get_parameter("Kt_lin").as_double();
        tau_ = std::max(1e-4, get_parameter("tau").as_double());
        deriv_tau_ = std::max(1e-4, get_parameter("deriv_tau").as_double());

        auto Kv = get_parameter("K").as_double_array();
        if (Kv.size() != 3)
            Kv = Kdef;
        K_ << Kv[0], Kv[1], Kv[2];
        omega0_rad_s_ = get_parameter("omega0_rad_s").as_double();

        topic_angle_deg_ = get_parameter("topic_angle_deg").as_string();
        topic_delta_vemf_ = get_parameter("topic_delta_vemf").as_string();
        topic_cmd_vel_ = get_parameter("topic_cmd_vel").as_string();

        // --- Build plant (continuous) ---
        // x=[theta, omega, Δv_emf], y=theta
        // θdot = ω
        // ωdot = (Kt_lin*La)/(Ja*Ke) * Δv_emf
        // (Δv_emf)dot = -Δv_emf/τ + (Ke/τ) * u,  u = motor speed [rad/s]
        A_.setZero();
        A_(0, 1) = 1.0;
        A_(1, 2) = (Kt_lin_ * La_) / (Ja_ * Ke_);
        A_(2, 2) = -1.0 / tau_;

        B_.setZero();
        B_(2) = Ke_ / tau_;

        C_ << 1.0, 0.0, 0.0;

        // Precompensator for unity DC gain
        computeKr();

        // --- I/O ---
        rclcpp::SensorDataQoS sensor_qos;

        sub_angle_ = create_subscription<std_msgs::msg::Float64>(
            topic_angle_deg_, sensor_qos,
            [this](const std_msgs::msg::Float64::SharedPtr m)
            {
                theta_deg_ = m->data;
                have_angle_ = true;
            });

        sub_delta_vemf_ = create_subscription<std_msgs::msg::Float64>(
            topic_delta_vemf_, sensor_qos,
            [this](const std_msgs::msg::Float64::SharedPtr m)
            {
                delta_vemf_ = m->data;
                have_vemf_ = true;
            });

        pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            topic_cmd_vel_, rclcpp::QoS(10).reliable());

        // Loop
        const auto T = std::chrono::duration<double>(1.0 / std::max(20.0, rate_hz_));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(T),
            std::bind(&AngleHoldSSController::loop, this));

        RCLCPP_INFO(get_logger(),
                    "K=[%.2f %.2f %.2f], Kr=%.5f, tau=%.4f, u∈[%.1f,%.1f], refs=%g deg",
                    K_(0), K_(1), K_(2), Kr_, tau_, u_min_, u_max_, theta_ref_deg_);
    }

    void AngleHoldSSController::computeKr()
    {
        Eigen::Matrix3d M = (A_ - B_ * K_);
        Eigen::FullPivLU<Eigen::Matrix3d> lu(M);
        if (!lu.isInvertible())
        {
            Kr_ = 0.0;
            RCLCPP_WARN(get_logger(), "A-BK not invertible; Kr=0");
            return;
        }
        double denom = (C_ * M.inverse() * B_)(0, 0);
        if (std::abs(denom) < 1e-9)
        {
            Kr_ = 0.0;
            RCLCPP_WARN(get_logger(), "C*(A-BK)^{-1}*B ≈ 0; Kr=0");
        }
        else
        {
            Kr_ = -1.0 / denom;
        }
    }

    void AngleHoldSSController::loop()
    {
        if (!(have_angle_ && have_vemf_))
            return;

        // State: theta(rad), omega via filtered derivative, Δv_emf(V)
        const double theta = theta_deg_ * M_PI / 180.0;
        auto now = this->get_clock()->now();

        if (have_prev_)
        {
            double dt = std::max(1e-4, (now - t_prev_).seconds());
            double omega_raw = (theta - theta_prev_) / dt;
            double alpha = dt / (deriv_tau_ + dt);
            omega_est_ += alpha * (omega_raw - omega_est_);
        }
        else
        {
            omega_est_ = 0.0;
            have_prev_ = true;
        }
        theta_prev_ = theta;
        t_prev_ = now;

        Vector3d x;
        x << theta, omega_est_, delta_vemf_;
        const double r = theta_ref_deg_ * M_PI / 180.0;

        double u_delta = omega0_rad_s_ * (Kr_ * r - (K_ * x)(0)); // desired motor speed [rad/s]
        double u_total = std::clamp(u_delta, u_min_, u_max_);     // prop: no negatives

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {u_total};
        pub_cmd_->publish(cmd);
    }

} // namespace prop_arm_ctrl

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<prop_arm_ctrl::AngleHoldSSController>());
    rclcpp::shutdown();
    return 0;
}

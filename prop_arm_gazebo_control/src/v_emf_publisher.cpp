#include "prop_arm_gazebo_control/v_emf_publisher.hpp"

#include <algorithm>
#include <cmath>

namespace prop_arm_ctrl
{

    VEmfPublisher::VEmfPublisher(const rclcpp::NodeOptions &opts)
        : rclcpp::Node("v_emf_publisher", opts)
    {
        // Parameters (declare + get)
        this->declare_parameter<double>("ke", 0.02);
        this->declare_parameter<double>("v_emf_eq", 0.0);
        this->declare_parameter<double>("lpf_tau", 0.01);
        this->declare_parameter<bool>("use_lpf", true);
        this->declare_parameter<std::string>("in_topic", "/prop_arm/motor_speed_est");
        this->declare_parameter<std::string>("out_v_emf_topic", "/prop_arm/v_emf");
        this->declare_parameter<std::string>("out_delta_topic", "/prop_arm/delta_v_emf");
        this->get_parameter("ke", ke_);
        this->get_parameter("v_emf_eq", v_emf_eq_);
        this->get_parameter("lpf_tau", lpf_tau_);
        this->get_parameter("use_lpf", use_lpf_);
        this->get_parameter("in_topic", in_topic_);
        this->get_parameter("out_v_emf_topic", out_v_emf_topic_);
        this->get_parameter("out_delta_topic", out_delta_topic_);

        // QoS: SensorData for high-rate streams
        rclcpp::SensorDataQoS sensor_qos;

        v_emf_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            out_v_emf_topic_, rclcpp::QoS(10).reliable());
        delta_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            out_delta_topic_, rclcpp::QoS(10).reliable());

        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            in_topic_, sensor_qos,
            std::bind(&VEmfPublisher::speedCb, this, std::placeholders::_1));
    }

    void VEmfPublisher::speedCb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Input: motor speed [rad/s]
        const double omega_m = msg->data;

        // Compute V_emf (raw)
        const double v_emf_raw = ke_ * omega_m; // [V]

        // Low-pass on V_emf for nice small-signal behavior
        double v_emf_now = v_emf_raw;
        const auto now = this->get_clock()->now();

        if (use_lpf_)
        {
            double dt = 0.0;
            if (have_last_)
            {
                dt = std::max(0.0, (now - last_stamp_).seconds());
            }
            // One-pole LPF: alpha = dt/(tau+dt), protect tau
            const double tau = std::max(1e-6, lpf_tau_);
            const double alpha = (dt <= 0.0) ? 1.0 : dt / (tau + dt);

            v_emf_filt_ = have_last_ ? (v_emf_filt_ + alpha * (v_emf_raw - v_emf_filt_)) : v_emf_raw;
            v_emf_now = v_emf_filt_;
        }
        else
        {
            v_emf_filt_ = v_emf_raw;
        }

        have_last_ = true;
        last_stamp_ = now;

        // ΔV_emf around chosen operating point
        const double delta_v = v_emf_now - v_emf_eq_;

        std_msgs::msg::Float64 out1;
        out1.data = v_emf_now;
        std_msgs::msg::Float64 out2;
        out2.data = delta_v;

        v_emf_pub_->publish(out1);
        delta_pub_->publish(out2);
    }

} // namespace prop_arm_ctrl

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    auto node = std::make_shared<prop_arm_ctrl::VEmfPublisher>(opts);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

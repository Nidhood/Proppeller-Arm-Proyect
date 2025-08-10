#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace prop_arm_ctrl
{

    class VEmfPublisher : public rclcpp::Node
    {
    public:
        explicit VEmfPublisher(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

    private:
        void speedCb(const std_msgs::msg::Float64::SharedPtr msg);

        // Params
        double ke_;                   // [V/(rad/s)]
        double v_emf_eq_;             // [V] equilibrium V_emf for small-signal ΔV_emf
        double lpf_tau_;              // [s] 1st-order low-pass time constant
        bool use_lpf_;                // enable/disable low-pass
        std::string in_topic_;        // motor speed [rad/s]
        std::string out_v_emf_topic_; // V_emf [V]
        std::string out_delta_topic_; // ΔV_emf [V]

        // I/O
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr v_emf_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delta_pub_;

        // State
        rclcpp::Time last_stamp_;
        bool have_last_{false};
        double v_emf_filt_{0.0};
    };

} // namespace prop_arm_ctrl

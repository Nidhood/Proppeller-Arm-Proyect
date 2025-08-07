#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <memory>

namespace prop_arm_control
{

    class PropellerForceController : public rclcpp::Node
    {
    public:
        PropellerForceController();
        ~PropellerForceController() = default;

    private:
        // PID Controller parameters
        double kp_;
        double ki_;
        double kd_;

        // Control state variables
        double integral_error_;
        double previous_error_;
        double target_angle_;
        double current_angle_;
        double current_velocity_;

        // Motor limits and control parameters
        double max_motor_force_;
        double min_motor_force_;
        double control_frequency_;
        double dt_;

        // ROS2 Publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_effort_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_velocity_pub_;

        // ROS2 Subscribers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_angle_sub_;

        // Timers
        rclcpp::TimerBase::SharedPtr control_timer_;
        rclcpp::TimerBase::SharedPtr startup_timer_;

        // Status tracking
        int log_counter_;
        bool joint_state_received_;
        int startup_counter_;
        bool auto_started_;

        // MIT reference frame conversion methods
        double mitToGazeboAngle(double mit_angle_degrees) const;
        double gazeboToMitAngle(double gazebo_radians) const;

        // Callback methods
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void target_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

        // Control methods
        void control_loop();
        void startup_diagnostics();
        void send_zero_commands();
    };

} // namespace prop_arm_control
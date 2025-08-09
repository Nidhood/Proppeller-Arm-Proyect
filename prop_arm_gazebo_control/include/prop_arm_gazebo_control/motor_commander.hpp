#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

#include <iostream>
#include <string>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <cmath>
#include <memory>

namespace prop_arm_control
{

    class MotorCommander : public rclcpp::Node
    {
    public:
        MotorCommander();
        ~MotorCommander() = default;

        // Main command methods
        void command_angle(double angle_degrees);
        void command_force(double force_newtons);
        void command_velocity(double velocity_rad_s);
        void stop_all_commands();

        // Utility methods
        void print_current_controllers();
        void stabilize_at_horizontal();

    private:
        enum class LastCommandType
        {
            NONE,
            POSITION,
            VELOCITY,
            EFFORT
        };

        std::unique_ptr<gz::transport::Node> gz_node_;

        // Publishers for different controller types
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

        // State tracking
        LastCommandType last_command_type_;

        // MIT reference frame conversion methods
        double mitToGazeboAngle(double mit_angle_degrees) const;
        double gazeboToMitAngle(double gazebo_radians) const;

        // Physics calculation methods
        double calculateThrustForAngle(double target_angle_degrees);

        // Internal utility methods
        void send_zero_commands();
    };

    // Utility functions
    void print_usage();

} // namespace prop_arm_control

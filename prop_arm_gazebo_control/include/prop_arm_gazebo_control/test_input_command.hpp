#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

namespace prop_arm_test_input
{
    enum class SignalType
    {
        STEP,
        IMPULSE,
        PARABOLA,
        RAMP
    };

    class TestInputCM : public rclcpp::Node
    {
    public:
        explicit TestInputCM(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

    private:
        void loop();
        double generateSignal(double t);

        // Parameters
        double rate_hz_{50.0};
        double start_time_{2.0};
        double duration_{5.0};
        double amplitude_{30.0};
        double impulse_width_{0.1};
        double hold_time_{2.0}; // Nuevo parámetro: tiempo en valor máximo
        SignalType signal_type_{SignalType::STEP};
        std::string signal_type_str_{"step"};

        // Topic names
        std::string topic_vref_{"/prop_arm/cmd/vref"};

        // ROS2 objects
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vref_;
        rclcpp::TimerBase::SharedPtr timer_;

        // State variables
        rclcpp::Time start_node_time_;
        bool started_{false};
    };

} // namespace prop_arm_test_input
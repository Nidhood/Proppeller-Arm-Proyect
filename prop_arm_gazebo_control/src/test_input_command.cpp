#include "prop_arm_gazebo_control/test_input_command.hpp"
#include <algorithm>
#include <cmath>

namespace prop_arm_test_input
{
    TestInputCM::TestInputCM(const rclcpp::NodeOptions &opts)
        : rclcpp::Node("test_input_command", opts)
    {
        declare_parameter("rate_hz", rate_hz_);
        declare_parameter("start_time", start_time_);
        declare_parameter("duration", duration_);
        declare_parameter("amplitude", amplitude_);
        declare_parameter("impulse_width", impulse_width_);
        declare_parameter("hold_time", hold_time_); // Nuevo parámetro
        declare_parameter("signal_type", signal_type_str_);
        declare_parameter("topic_vref", topic_vref_);

        rate_hz_ = get_parameter("rate_hz").as_double();
        start_time_ = get_parameter("start_time").as_double();
        duration_ = get_parameter("duration").as_double();
        amplitude_ = get_parameter("amplitude").as_double();
        impulse_width_ = get_parameter("impulse_width").as_double();
        hold_time_ = get_parameter("hold_time").as_double(); // Nuevo parámetro
        signal_type_str_ = get_parameter("signal_type").as_string();
        topic_vref_ = get_parameter("topic_vref").as_string();

        if (signal_type_str_ == "step")
        {
            signal_type_ = SignalType::STEP;
        }
        else if (signal_type_str_ == "impulse")
        {
            signal_type_ = SignalType::IMPULSE;
        }
        else if (signal_type_str_ == "parabola")
        {
            signal_type_ = SignalType::PARABOLA;
        }
        else if (signal_type_str_ == "ramp")
        {
            signal_type_ = SignalType::RAMP;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unknown signal type '%s', using STEP", signal_type_str_.c_str());
            signal_type_ = SignalType::STEP;
        }

        rate_hz_ = std::max(1.0, rate_hz_);
        start_time_ = std::max(0.0, start_time_);
        duration_ = std::max(0.1, duration_);
        impulse_width_ = std::max(0.01, impulse_width_);
        hold_time_ = std::max(0.0, hold_time_); // Validación del nuevo parámetro

        pub_vref_ = create_publisher<std_msgs::msg::Float64>(
            topic_vref_, rclcpp::QoS(10).reliable());

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TestInputCM::loop, this));

        start_node_time_ = this->get_clock()->now();

        RCLCPP_INFO(get_logger(),
                    "Test Input Command Node initialized:");
        RCLCPP_INFO(get_logger(),
                    "  Signal: %s, Amplitude: %.1f°, Start: %.1fs, Duration: %.1fs, Hold: %.1fs",
                    signal_type_str_.c_str(), amplitude_, start_time_, duration_, hold_time_);
        RCLCPP_INFO(get_logger(),
                    "  Publishing to: %s at %.1f Hz",
                    topic_vref_.c_str(), rate_hz_);
    }

    void TestInputCM::loop()
    {
        auto current_time = this->get_clock()->now();
        double elapsed_time = (current_time - start_node_time_).seconds();

        double signal_value = generateSignal(elapsed_time);

        std_msgs::msg::Float64 msg;
        msg.data = signal_value;
        pub_vref_->publish(msg);

        if (!started_ && signal_value != 0.0)
        {
            RCLCPP_INFO(get_logger(), "Signal activated at t=%.2f s", elapsed_time);
            started_ = true;
        }
    }

    double TestInputCM::generateSignal(double t)
    {
        if (t < start_time_)
        {
            return 0.0;
        }

        double relative_time = t - start_time_;

        switch (signal_type_)
        {
        case SignalType::STEP:
            return (relative_time <= duration_) ? amplitude_ : 0.0;

        case SignalType::IMPULSE:
            return (relative_time <= impulse_width_) ? amplitude_ : 0.0;

        case SignalType::PARABOLA:
            if (relative_time <= duration_)
            {
                // Fase de crecimiento parabólico
                double normalized_time = relative_time / duration_;
                return amplitude_ * normalized_time * normalized_time;
            }
            else if (relative_time <= duration_ + hold_time_)
            {
                // Fase de mantenimiento en valor máximo
                return amplitude_;
            }
            else
            {
                // Señal se apaga después del tiempo de mantenimiento
                return 0.0;
            }

        case SignalType::RAMP:
            if (relative_time <= duration_)
            {
                // Fase de crecimiento lineal
                return amplitude_ * (relative_time / duration_);
            }
            else if (relative_time <= duration_ + hold_time_)
            {
                // Fase de mantenimiento en valor máximo
                return amplitude_;
            }
            else
            {
                // Señal se apaga después del tiempo de mantenimiento
                return 0.0;
            }

        default:
            return 0.0;
        }
    }
} // namespace prop_arm_test_input

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<prop_arm_test_input::TestInputCM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
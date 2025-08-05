#ifndef PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_
#define PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

// Hardware interface headers
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"

// ROS headers
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

// Gazebo headers
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

namespace prop_arm_gazebo_control
{

class PropArmHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

    /// \brief Initialize the hardware from the information in the URDF/ros2_control XACRO
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    /// \brief Export StateInterface for each joint (position, velocity)
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /// \brief Export CommandInterface for motor speed (not joint position!)
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /// \brief Called when the controller_manager activates this hardware
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    /// \brief Called when the controller_manager deactivates this hardware
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    /// \brief Read the current state from Gazebo
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /// \brief Send motor speed commands to Gazebo
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // Joint data storage
    struct JointData
    {
        double position{0.0};
        double velocity{0.0};
        double motor_speed_command{0.0};  // Comando de velocidad del motor
    };

    std::unordered_map<std::string, JointData> joints_;
    
    // Gazebo communication
    std::unique_ptr<gz::transport::Node> gz_node_;
    gz::transport::Node::Publisher actuators_pub_;
    
    // Robot namespace for topic names
    std::string robot_namespace_;
    std::string actuators_topic_;
    
    // PID controller for converting position commands to motor speeds
    struct PIDController
    {
        double kp{100.0};
        double ki{0.1};
        double kd{10.0};
        double prev_error{0.0};
        double integral{0.0};
        double output_min{-785.0};  // Matching maxRotVelocity
        double output_max{785.0};
    };
    
    std::unordered_map<std::string, PIDController> pid_controllers_;
};

} // namespace prop_arm_gazebo_control

#endif // PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_
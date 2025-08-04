#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace prop_arm_gazebo_control
{

    hardware_interface::CallbackReturn PropArmHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        angle_ = 0.0;
        velocity_ = 0.0;
        command_ = 0.0;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PropArmHardware::export_state_interfaces()
    {
        return {
            hardware_interface::StateInterface("arm_link_joint", hardware_interface::HW_IF_POSITION, &angle_),
            hardware_interface::StateInterface("arm_link_joint", hardware_interface::HW_IF_VELOCITY, &velocity_)};
    }

    std::vector<hardware_interface::CommandInterface> PropArmHardware::export_command_interfaces()
    {
        return {
            hardware_interface::CommandInterface("arm_link_joint", hardware_interface::HW_IF_POSITION, &command_)};
    }

    hardware_interface::CallbackReturn PropArmHardware::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type PropArmHardware::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        angle_ = command_; // For testing purposes; in simulation, actual state would be read from Gazebo
        velocity_ = 0.0;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropArmHardware::write(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Send 'command_' to Gazebo (via GazeboSimSystemInterface mechanisms)
        return hardware_interface::return_type::OK;
    }

} // namespace prop_arm_gazebo_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(prop_arm_gazebo_control::PropArmHardware, gz_ros2_control::GazeboSimSystemInterface)

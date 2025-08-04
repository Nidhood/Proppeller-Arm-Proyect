#ifndef PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_
#define PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_

// License omitted for brevity

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace prop_arm_gazebo_control
{

    class PropArmHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

        /** \brief Initialize the hardware from the information in the URDF/ros2_control XACRO */
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        /** \brief Export StateInterface for each joint (position, velocity) */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /** \brief Export CommandInterface for each joint (position) */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /** \brief Called when the controller_manager activates this hardware */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /** \brief Called when the controller_manager deactivates this hardware */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        /** \brief Read the current state from the simulation (or real HW) */
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /** \brief Send commands to the simulation (or real HW) */
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        double angle_{0.0};
        double velocity_{0.0};
        double command_{0.0};
    };

} // namespace prop_arm_gazebo_control

#endif // PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_

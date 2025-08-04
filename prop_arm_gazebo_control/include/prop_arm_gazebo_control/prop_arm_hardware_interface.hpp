#ifndef PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_
#define PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"

#include "gz_ros2_control/gz_ros2_control_plugin.hpp"

namespace prop_arm_gazebo_control
{

    class PropArmHardware : public gz_ros2_control::GazeboSimSystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        double angle_ = 0.0;
        double velocity_ = 0.0;
        double command_ = 0.0;
    };

} // namespace prop_arm_gazebo_control

#endif // PROP_ARM_GAZEBO_CONTROL__PROP_ARM_HARDWARE_INTERFACE_HPP_

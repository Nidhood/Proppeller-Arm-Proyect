#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

namespace sim = gz::sim;

namespace prop_arm_gazebo_control
{

    struct JointData
    {
        double position{0.0};                    // [rad]
        double velocity{0.0};                    // [rad/s]
        double velocity_command{0.0};            // [rad/s]
        sim::Entity sim_joint{sim::kNullEntity}; // Simulation joint entity
    };

    class PropArmHardware : public gz_ros2_control::GazeboSimSystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

        bool initSim(rclcpp::Node::SharedPtr &model_nh,
                     std::map<std::string, sim::Entity> &joints,
                     const hardware_interface::HardwareInfo &hardware_info,
                     sim::EntityComponentManager &ecm,
                     unsigned int update_rate) override;

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &period) override;

    private:
        // Config
        std::string robot_namespace_{"prop_arm"};
        std::string actuators_topic_{"/prop_arm/command/motor_speed"};
        int actuator_index_{0};       // index into Actuators.velocity[]
        double max_rot_vel_{785.0};   // [rad/s]
        double motor_speed_est_{0.0}; // rad/s
        double tau_up_{0.01};         // s
        double tau_down_{0.005};      // s

        // ROS / Gazebo
        rclcpp::Node::SharedPtr nh_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_speed_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_angle_pub_;
        sim::EntityComponentManager *ecm_{nullptr};
        std::map<std::string, sim::Entity> enabled_joints_;
        std::unique_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher actuators_pub_;

        // Interfaces
        std::unordered_map<std::string, JointData> joints_;
        std::vector<hardware_interface::StateInterface> state_interfaces_;
        std::vector<hardware_interface::CommandInterface> command_interfaces_;
    };

} // namespace prop_arm_gazebo_control

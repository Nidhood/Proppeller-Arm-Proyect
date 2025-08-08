#ifndef PROP_ARM_HARDWARE_INTERFACE_HPP_
#define PROP_ARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <cmath>

// Use gz_ros2_control base class:
#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Gazebo simulation components:
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>

// Gazebo transport:
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

namespace sim = gz::sim;

namespace prop_arm_gazebo_control
{
    // Structure to hold joint data based on the electro-mechanical model:
    struct JointData
    {
        double position = 0.0;         // θ_a - arm angle in MIT frame (0=horizontal).
        double velocity = 0.0;         // ω_a - arm angular velocity.
        double effort = 0.0;           // Motor effort/torque.
        double position_command = 0.0; // Desired arm position in MIT frame (0=horizontal),
        double velocity_command = 0.0; // Direct velocity command.
        double effort_command = 0.0;   // Direct force/thrust command.

        // Command tracking to fix "stuck" command issue
        double last_position_command = 0.0;
        double last_velocity_command = 0.0;
        double last_effort_command = 0.0;

        // Gazebo simulation entity:
        sim::Entity sim_joint = sim::kNullEntity;
    };

    // PID Controller for position control
    struct PIDController
    {
        double kp = 50.0;           // Proportional gain.
        double ki = 0.1;            // Small integral gain.
        double kd = 5.0;            // Derivative gain for damping.
        double integral = 0.0;      // Integral term for anti-windup.
        double prev_error = 0.0;    // Previous error for derivative calculation.
        double output_min = -785.0; // Max motor speed (negative).
        double output_max = 785.0;  // Max motor speed (positive).
    };

    // Electro-mechanical model parameters based on MIT model:
    struct ModelParameters
    {
        double Ja = 0.1;                      // Arm inertia.
        double Jm = 0.01;                     // Motor inertia.
        double Kt = 0.1;                      // Motor torque constant.
        double Ke = 0.1;                      // Motor EMF constant.
        double Rm = 1.0;                      // Motor resistance.
        double Ra = 0.1;                      // Arm resistance/damping.
        double Kf = 0.05;                     // Friction coefficient.
        double force_to_velocity_gain = 10.0; // Converts force commands to velocity
    };

    // Inherit from GazeboSimSystemInterface:
    class PropArmHardware : public gz_ros2_control::GazeboSimSystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

        // gz_ros2_control::GazeboSimSystemInterface methods:
        bool initSim(
            rclcpp::Node::SharedPtr &model_nh,
            std::map<std::string, sim::Entity> &joints,
            const hardware_interface::HardwareInfo &hardware_info,
            sim::EntityComponentManager &_ecm,
            unsigned int update_rate) override;

        // hardware_interface::SystemInterface methods:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // Hardware interface data:
        std::unordered_map<std::string, JointData> joints_;
        std::unordered_map<std::string, PIDController> pid_controllers_;
        ModelParameters model_params_;

        // Gazebo simulation data:
        sim::EntityComponentManager *ecm_{nullptr};
        std::map<std::string, sim::Entity> enabled_joints_;
        unsigned int update_rate_{100};
        rclcpp::Node::SharedPtr nh_;

        // Gazebo communication:
        std::unique_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher actuators_pub_;
        std::string actuators_topic_;
        std::string robot_namespace_;

        // State and command interfaces storage
        std::vector<hardware_interface::StateInterface> state_interfaces_;
        std::vector<hardware_interface::CommandInterface> command_interfaces_;

        // Angle transformation methods
        double mitToGazeboAngle(double mit_angle) const;
        double gazeboToMitAngle(double gazebo_angle) const;

        // Electro-mechanical model simulation
        void updateElectroMechanicalModel(const std::string &joint_name,
                                          double motor_speed_cmd,
                                          double dt);
    };

} // namespace prop_arm_gazebo_control

#endif // PROP_ARM_HARDWARE_INTERFACE_HPP_
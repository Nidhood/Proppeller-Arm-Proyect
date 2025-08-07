#pragma once

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
    /**
     * @brief Structure to hold joint data based on the MIT electro-mechanical model
     *
     * Contains all the state and command variables for a single joint,
     * following MIT's reference frame where 0° = horizontal position.
     */
    struct JointData
    {
        // State variables (read from simulation)
        double position = 0.0; ///< θ_a - arm angle in MIT frame (0=horizontal, +90=up, -90=down)
        double velocity = 0.0; ///< ω_a - arm angular velocity (rad/s)
        double effort = 0.0;   ///< Motor effort/torque (N·m)

        // Command variables (written by controllers)
        double position_command = 0.0; ///< Desired arm position in MIT frame (rad)
        double velocity_command = 0.0; ///< Direct velocity command (rad/s)
        double effort_command = 0.0;   ///< Direct force/thrust command (N)

        // Gazebo simulation entity
        sim::Entity sim_joint = sim::kNullEntity;
    };

    /**
     * @brief PID Controller for position control (MIT primary objective)
     *
     * Implements PID control for precise angle control using propeller thrust.
     * This is the main control mode for the MIT PropArm project.
     */
    struct PIDController
    {
        // PID gains - tuned for MIT PropArm dynamics
        double kp = 100.0; ///< Proportional gain for position error
        double ki = 5.0;   ///< Integral gain for steady-state error elimination
        double kd = 15.0;  ///< Derivative gain for damping oscillations

        // PID state variables
        double integral = 0.0;   ///< Accumulated integral term
        double prev_error = 0.0; ///< Previous error for derivative calculation

        // Output limits (motor speed constraints)
        double output_min = -785.0; ///< Maximum motor speed (reverse direction)
        double output_max = 785.0;  ///< Maximum motor speed (forward direction)
    };

    /**
     * @brief Electro-mechanical model parameters based on MIT PropArm specifications
     *
     * Contains all the physical parameters for the MIT PropArm system,
     * implementing the electro-mechanical coupling between motor and arm.
     */
    struct ModelParameters
    {
        // System inertias (kg·m²)
        double Ja = 0.05;  ///< Arm inertia - reduced for responsive control
        double Jm = 0.001; ///< Motor inertia - small for propeller dynamics

        // Motor electrical constants
        double Kt = 0.05; ///< Motor torque constant (N·m/A)
        double Ke = 0.05; ///< Motor EMF constant (V·s/rad)
        double Rm = 2.0;  ///< Motor resistance (Ω)

        // Mechanical friction and damping
        double Ra = 0.05; ///< Arm resistance/damping coefficient (N·m·s/rad)
        double Kf = 0.01; ///< Nonlinear friction coefficient (N·m·s²/rad²)

        // Propeller model parameters (from SDF configuration)
        double motorConstant = 0.008; ///< Motor thrust constant: F = k·ω² (N/(rad/s)²)
        double momentConstant = 0.02; ///< Moment arm constant (m)

        // Control conversion gains
        double force_to_velocity_gain = 5.0; ///< Converts force commands to velocity
    };

    /**
     * @brief Hardware interface for MIT PropArm in Gazebo simulation
     *
     * Implements the ros2_control hardware interface for the MIT PropArm project.
     * Inherits from GazeboSimSystemInterface to integrate with Gazebo simulation.
     *
     * Key features:
     * - MIT reference frame: 0° = horizontal, +90° = up, -90° = down
     * - Multiple control modes: position (primary), velocity, force/effort
     * - Electro-mechanical model implementation
     * - PID control for precise angle positioning
     */
    class PropArmHardware : public gz_ros2_control::GazeboSimSystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

        // Constructor and destructor
        PropArmHardware() = default;
        virtual ~PropArmHardware() = default;

        // gz_ros2_control::GazeboSimSystemInterface interface methods
        bool initSim(
            rclcpp::Node::SharedPtr &model_nh,
            std::map<std::string, sim::Entity> &joints,
            const hardware_interface::HardwareInfo &hardware_info,
            sim::EntityComponentManager &_ecm,
            unsigned int update_rate) override;

        // hardware_interface::SystemInterface interface methods
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
        // Core data structures
        std::unordered_map<std::string, JointData> joints_;
        std::unordered_map<std::string, PIDController> pid_controllers_;
        ModelParameters model_params_;

        // Gazebo simulation integration
        sim::EntityComponentManager *ecm_{nullptr};
        std::map<std::string, sim::Entity> enabled_joints_;
        unsigned int update_rate_{100};
        rclcpp::Node::SharedPtr nh_;

        // Gazebo transport communication
        std::unique_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher actuators_pub_;
        std::string actuators_topic_;
        std::string robot_namespace_;

        // Hardware interface storage
        std::vector<hardware_interface::StateInterface> state_interfaces_;
        std::vector<hardware_interface::CommandInterface> command_interfaces_;

        /**
         * @brief Convert MIT reference frame angle to Gazebo joint frame
         * @param mit_angle Angle in MIT frame (radians, 0=horizontal)
         * @return Angle in Gazebo joint frame (radians)
         */
        double mitToGazeboAngle(double mit_angle) const;

        /**
         * @brief Convert Gazebo joint frame angle to MIT reference frame
         * @param gazebo_angle Angle in Gazebo joint frame (radians)
         * @return Angle in MIT frame (radians, 0=horizontal)
         */
        double gazeboToMitAngle(double gazebo_angle) const;

        /**
         * @brief Update the MIT electro-mechanical model simulation
         * @param joint_name Name of the joint to update
         * @param motor_speed_cmd Motor speed command (rad/s)
         * @param dt Time step for integration (seconds)
         */
        void updateElectroMechanicalModel(const std::string &joint_name,
                                          double motor_speed_cmd,
                                          double dt);
    };

} // namespace prop_arm_gazebo_control
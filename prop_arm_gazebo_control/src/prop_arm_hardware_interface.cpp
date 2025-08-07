#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"

// Gazebo simulation components
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/Name.hh>

namespace prop_arm_gazebo_control
{

    bool PropArmHardware::initSim(
        rclcpp::Node::SharedPtr &model_nh,
        std::map<std::string, sim::Entity> &joints,
        const hardware_interface::HardwareInfo &hardware_info,
        sim::EntityComponentManager &_ecm,
        unsigned int update_rate)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Initializing PropArm Hardware Interface");

        this->nh_ = model_nh;
        this->ecm_ = &_ecm;
        this->enabled_joints_ = joints;
        this->update_rate_ = update_rate;

        // Get robot namespace from hardware info
        robot_namespace_ = "prop_arm"; // Default
        auto param_it = hardware_info.hardware_parameters.find("robot_namespace");
        if (param_it != hardware_info.hardware_parameters.end())
        {
            robot_namespace_ = param_it->second;
        }

        // Initialize Gazebo transport for MulticopterMotorModel plugin
        gz_node_ = std::make_unique<gz::transport::Node>();

        // Setup actuators topic for propeller - this matches your MulticopterMotorModel plugin
        actuators_topic_ = "/" + robot_namespace_ + "/command/motor_speed";
        actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

        // Wait for publisher to be ready
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Initialize joint data and controllers for each joint in hardware_info
        for (const auto &joint_info : hardware_info.joints)
        {
            const std::string &joint_name = joint_info.name;

            // Check if this joint exists in the enabled joints from Gazebo
            auto it = enabled_joints_.find(joint_name);
            if (it == enabled_joints_.end())
            {
                RCLCPP_WARN(this->nh_->get_logger(),
                            "Joint '%s' not found in Gazebo model", joint_name.c_str());
                continue;
            }

            // Initialize joint data
            joints_[joint_name] = JointData{};
            joints_[joint_name].sim_joint = it->second;
            pid_controllers_[joint_name] = PIDController{};

            // Create necessary Gazebo components for this joint
            sim::Entity joint_entity = it->second;

            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointPosition().TypeId()))
            {
                _ecm.CreateComponent(joint_entity, sim::components::JointPosition());
            }

            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointVelocity().TypeId()))
            {
                _ecm.CreateComponent(joint_entity, sim::components::JointVelocity());
            }

            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointVelocityCmd().TypeId()))
            {
                _ecm.CreateComponent(joint_entity, sim::components::JointVelocityCmd());
            }

            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointForceCmd().TypeId()))
            {
                _ecm.CreateComponent(joint_entity, sim::components::JointForceCmd());
                _ecm.SetComponentData<sim::components::JointForceCmd>(joint_entity, std::vector<double>{0.0});
            }

            // Setup state interfaces
            for (const auto &interface : joint_info.state_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    state_interfaces_.emplace_back(
                        joint_name, hardware_interface::HW_IF_POSITION, &joints_[joint_name].position);
                }
                else if (interface.name == hardware_interface::HW_IF_VELOCITY)
                {
                    state_interfaces_.emplace_back(
                        joint_name, hardware_interface::HW_IF_VELOCITY, &joints_[joint_name].velocity);
                }
                else if (interface.name == hardware_interface::HW_IF_EFFORT)
                {
                    state_interfaces_.emplace_back(
                        joint_name, hardware_interface::HW_IF_EFFORT, &joints_[joint_name].effort);
                }
            }

            // Setup command interfaces
            for (const auto &interface : joint_info.command_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    command_interfaces_.emplace_back(
                        joint_name, hardware_interface::HW_IF_POSITION, &joints_[joint_name].position_command);
                }
                else if (interface.name == hardware_interface::HW_IF_VELOCITY)
                {
                    command_interfaces_.emplace_back(
                        joint_name, hardware_interface::HW_IF_VELOCITY, &joints_[joint_name].velocity_command);
                }
                else if (interface.name == hardware_interface::HW_IF_EFFORT)
                {
                    command_interfaces_.emplace_back(
                        joint_name, hardware_interface::HW_IF_EFFORT, &joints_[joint_name].effort_command);
                }
            }

            RCLCPP_INFO(this->nh_->get_logger(),
                        "Initialized joint: %s with %zu state and %zu command interfaces",
                        joint_name.c_str(), joint_info.state_interfaces.size(), joint_info.command_interfaces.size());
        }

        RCLCPP_INFO(this->nh_->get_logger(),
                    "Actuators command topic: %s", actuators_topic_.c_str());
        RCLCPP_INFO(this->nh_->get_logger(),
                    "Hardware interface initialized with %zu joints", joints_.size());

        return true;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "PropArm Hardware initialized");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    PropArmHardware::export_state_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                    "Exported %zu state interfaces", state_interfaces_.size());
        return std::move(state_interfaces_);
    }

    std::vector<hardware_interface::CommandInterface>
    PropArmHardware::export_command_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                    "Exported %zu command interfaces", command_interfaces_.size());
        return std::move(command_interfaces_);
    }

    hardware_interface::CallbackReturn PropArmHardware::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Activating PropArm Hardware");

        // Reset PID controllers
        for (auto &[joint_name, pid] : pid_controllers_)
        {
            pid.prev_error = 0.0;
            pid.integral = 0.0;
        }

        // Initialize joint commands - start at horizontal position (0 degrees MIT frame)
        for (auto &[joint_name, joint_data] : joints_)
        {
            joint_data.position = 0.0; // This will be updated from Gazebo
            joint_data.velocity = 0.0;
            joint_data.effort = 0.0;
            joint_data.velocity_command = 0.0;
            joint_data.position_command = 0.0; // MIT reference frame (0 = horizontal)
            joint_data.effort_command = 0.0;
        }

        // Send initial zero command to motors
        gz::msgs::Actuators actuators_msg;
        actuators_msg.add_velocity(0.0);
        if (!actuators_pub_.Publish(actuators_msg))
        {
            RCLCPP_WARN(rclcpp::get_logger("PropArmHardware"),
                        "Failed to publish initial actuator command");
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Deactivating PropArm Hardware");

        // Stop the motor
        gz::msgs::Actuators actuators_msg;
        actuators_msg.add_velocity(0.0);
        actuators_pub_.Publish(actuators_msg);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Convert from MIT reference frame (0=horizontal) to Gazebo joint frame
    double PropArmHardware::mitToGazeboAngle(double mit_angle) const
    {
        // MIT: 0 = horizontal, positive = up
        // Gazebo (with corrected URDF): 0 = horizontal, positive = up
        // No transformation needed with corrected URDF
        return mit_angle;
    }

    // Convert from Gazebo joint frame to MIT reference frame
    double PropArmHardware::gazeboToMitAngle(double gazebo_angle) const
    {
        // Gazebo (with corrected URDF): 0 = horizontal, positive = up
        // MIT: 0 = horizontal, positive = up
        // No transformation needed with corrected URDF
        return gazebo_angle;
    }

    void PropArmHardware::updateElectroMechanicalModel(const std::string &joint_name,
                                                       double motor_command,
                                                       double dt)
    {
        auto &joint = joints_[joint_name];

        // MIT Electro-mechanical model implementation
        // State variables: θₐ (arm angle), ωₐ (arm angular velocity)
        double theta_a = joint.position; // Already in MIT frame
        double omega_a = joint.velocity;

        // Motor dynamics based on thrust/force command
        // Convert force command to equivalent motor speed using the model
        double motor_speed = 0.0;

        if (std::abs(joint.effort_command) > 1e-6)
        {
            // Force mode: F = Kt * ω_m / Rm (simplified)
            motor_speed = joint.effort_command * model_params_.force_to_velocity_gain;
        }
        else if (std::abs(joint.velocity_command) > 1e-6)
        {
            // Direct velocity mode
            motor_speed = joint.velocity_command;
        }
        else
        {
            // Position control mode - use PID output
            motor_speed = motor_command;
        }

        // Arm dynamics with electro-mechanical coupling
        // Torque from motor: τ = Kt * i, where i relates to motor speed
        double motor_torque = model_params_.Kt * motor_speed / model_params_.Rm;

        // Arm equation: Ja * ωₐ_dot = τ_motor - Ra * ωₐ - Kf * sign(ωₐ) * ωₐ²
        double friction_torque = model_params_.Kf * std::copysign(1.0, omega_a) * omega_a * omega_a;
        double damping_torque = model_params_.Ra * omega_a;

        double omega_a_dot = (motor_torque - damping_torque - friction_torque) / model_params_.Ja;

        // Integration (Euler method)
        joint.velocity += omega_a_dot * dt;
        joint.position += joint.velocity * dt;

        // Update effort (torque) for state interface
        joint.effort = motor_torque;

        // Apply limits and normalization
        joint.position = std::fmod(joint.position + M_PI, 2.0 * M_PI) - M_PI;

        // Velocity damping to prevent unrealistic accelerations
        joint.velocity *= 0.999;
    }

    hardware_interface::return_type PropArmHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // Read from Gazebo simulation
        for (auto &[joint_name, joint_data] : joints_)
        {
            if (joint_data.sim_joint == sim::kNullEntity)
            {
                continue;
            }

            // Get joint position from Gazebo and convert to MIT frame
            const auto *jointPositions =
                ecm_->Component<sim::components::JointPosition>(joint_data.sim_joint);
            if (jointPositions && jointPositions->Data().size() > 0)
            {
                double gazebo_angle = jointPositions->Data()[0];
                joint_data.position = gazeboToMitAngle(gazebo_angle);
            }

            // Get joint velocity from Gazebo
            const auto *jointVelocity =
                ecm_->Component<sim::components::JointVelocity>(joint_data.sim_joint);
            if (jointVelocity && jointVelocity->Data().size() > 0)
            {
                joint_data.velocity = jointVelocity->Data()[0];
            }

            // For effort, we'll use our model calculation rather than Gazebo's
            // since we're implementing the electro-mechanical model
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropArmHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        for (auto &[joint_name, joint_data] : joints_)
        {
            if (joint_data.sim_joint == sim::kNullEntity)
            {
                continue;
            }

            double motor_command = 0.0;
            bool command_received = false;

            // Priority: Effort > Velocity > Position
            if (std::abs(joint_data.effort_command) > 1e-6)
            {
                // Force/Effort mode - convert to motor speed
                motor_command = joint_data.effort_command * model_params_.force_to_velocity_gain;
                command_received = true;

                RCLCPP_DEBUG(rclcpp::get_logger("PropArmHardware"),
                             "Force mode: effort=%.2f -> motor_speed=%.2f",
                             joint_data.effort_command, motor_command);
            }
            else if (std::abs(joint_data.velocity_command) > 1e-6)
            {
                // Direct velocity mode
                motor_command = joint_data.velocity_command;
                command_received = true;

                RCLCPP_DEBUG(rclcpp::get_logger("PropArmHardware"),
                             "Velocity mode: command=%.2f", motor_command);
            }
            else if (std::abs(joint_data.position_command - joint_data.position) > 1e-6)
            {
                // Position control mode using PID
                auto &pid = pid_controllers_[joint_name];

                // Commands come in MIT frame, no conversion needed
                double desired_position = joint_data.position_command;
                double current_position = joint_data.position;

                // Calculate position error with angle wrapping
                double error = desired_position - current_position;
                while (error > M_PI)
                    error -= 2.0 * M_PI;
                while (error < -M_PI)
                    error += 2.0 * M_PI;

                // PID calculation
                pid.integral += error * period.seconds();

                // Anti-windup
                if (std::abs(pid.integral) > 1.0)
                {
                    pid.integral = std::copysign(1.0, pid.integral);
                }

                double derivative = 0.0;
                if (period.seconds() > 0.0)
                {
                    derivative = (error - pid.prev_error) / period.seconds();
                }

                motor_command = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

                // Limit output
                motor_command = std::max(pid.output_min, std::min(pid.output_max, motor_command));

                pid.prev_error = error;
                command_received = true;

                RCLCPP_DEBUG(rclcpp::get_logger("PropArmHardware"),
                             "Position mode: target=%.2f, current=%.2f, error=%.2f -> motor_speed=%.2f",
                             desired_position, current_position, error, motor_command);
            }

            if (command_received)
            {
                // Convert MIT angle command to Gazebo frame for joint control
                double gazebo_command = mitToGazeboAngle(motor_command);

                // Apply motor command to Gazebo joint
                ecm_->SetComponentData<gz::sim::components::JointVelocityCmd>(
                    joint_data.sim_joint, {gazebo_command});

                // Send command to MulticopterMotorModel plugin for propeller thrust
                gz::msgs::Actuators actuators_msg;
                actuators_msg.add_velocity(motor_command);

                if (!actuators_pub_.Publish(actuators_msg))
                {
                    static int error_count = 0;
                    if (++error_count % 100 == 0)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("PropArmHardware"),
                                    "Failed to publish actuator command (count: %d)", error_count);
                    }
                }

                // Update electro-mechanical model
                updateElectroMechanicalModel(joint_name, motor_command, period.seconds());
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace prop_arm_gazebo_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    prop_arm_gazebo_control::PropArmHardware,
    gz_ros2_control::GazeboSimSystemInterface)
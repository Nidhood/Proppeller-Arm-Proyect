#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>

namespace prop_arm_gazebo_control
{

    bool PropArmHardware::initSim(
        rclcpp::Node::SharedPtr &model_nh,
        std::map<std::string, sim::Entity> &joints,
        const hardware_interface::HardwareInfo &hardware_info,
        sim::EntityComponentManager &_ecm,
        unsigned int update_rate)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Initializing MIT PropArm Hardware Interface");

        this->nh_ = model_nh;
        this->ecm_ = &_ecm;
        this->enabled_joints_ = joints;
        this->update_rate_ = update_rate;

        robot_namespace_ = "prop_arm";

        // Initialize Gazebo transport for motor commands
        gz_node_ = std::make_unique<gz::transport::Node>();

        // CRITICAL FIX: Use the correct topic that matches your MulticopterMotorModel plugin
        actuators_topic_ = "/" + robot_namespace_ + "/command/motor_speed";

        // Create publisher for motor commands
        actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

        // Wait for publisher to be ready
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!actuators_pub_.Valid())
        {
            RCLCPP_ERROR(this->nh_->get_logger(), "Failed to advertise to %s", actuators_topic_.c_str());
            return false;
        }

        // Initialize joints and controllers
        for (const auto &joint_info : hardware_info.joints)
        {
            const std::string &joint_name = joint_info.name;

            auto it = enabled_joints_.find(joint_name);
            if (it == enabled_joints_.end())
            {
                RCLCPP_WARN(this->nh_->get_logger(), "Joint '%s' not found", joint_name.c_str());
                continue;
            }

            joints_[joint_name] = JointData{};
            joints_[joint_name].sim_joint = it->second;
            pid_controllers_[joint_name] = PIDController{};

            // MIT PropArm specific PID tuning for angle control
            pid_controllers_[joint_name].kp = 200.0; // Strong proportional for angle tracking
            pid_controllers_[joint_name].ki = 15.0;  // Moderate integral to eliminate steady-state error
            pid_controllers_[joint_name].kd = 35.0;  // Good damping to prevent oscillations

            // Motor speed limits (positive only - thrust is always upward)
            pid_controllers_[joint_name].output_min = 0.0;   // No negative thrust
            pid_controllers_[joint_name].output_max = 785.0; // Max motor speed

            sim::Entity joint_entity = it->second;

            // Ensure required components exist
            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointPosition().TypeId()))
            {
                _ecm.CreateComponent(joint_entity, sim::components::JointPosition());
            }
            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointVelocity().TypeId()))
            {
                _ecm.CreateComponent(joint_entity, sim::components::JointVelocity());
            }

            // Setup state interfaces
            for (const auto &interface : joint_info.state_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &joints_[joint_name].position);
                }
                else if (interface.name == hardware_interface::HW_IF_VELOCITY)
                {
                    state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &joints_[joint_name].velocity);
                }
                else if (interface.name == hardware_interface::HW_IF_EFFORT)
                {
                    state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &joints_[joint_name].effort);
                }
            }

            // Setup command interfaces
            for (const auto &interface : joint_info.command_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &joints_[joint_name].position_command);
                }
                else if (interface.name == hardware_interface::HW_IF_VELOCITY)
                {
                    command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &joints_[joint_name].velocity_command);
                }
                else if (interface.name == hardware_interface::HW_IF_EFFORT)
                {
                    command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &joints_[joint_name].effort_command);
                }
            }

            RCLCPP_INFO(this->nh_->get_logger(), "Initialized joint: %s", joint_name.c_str());
        }

        RCLCPP_INFO(this->nh_->get_logger(), "Motor command topic: %s", actuators_topic_.c_str());
        RCLCPP_INFO(this->nh_->get_logger(), "Hardware interface ready with %zu joints", joints_.size());

        return true;
    }

    hardware_interface::return_type PropArmHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (auto &[joint_name, joint_data] : joints_)
        {
            if (joint_data.sim_joint == sim::kNullEntity)
                continue;

            // Read joint position (arm angle)
            const auto *jointPositions = ecm_->Component<sim::components::JointPosition>(joint_data.sim_joint);
            if (jointPositions && jointPositions->Data().size() > 0)
            {
                // Store in MIT frame: 0 = horizontal, positive = up
                joint_data.position = jointPositions->Data()[0];
            }

            // Read joint velocity
            const auto *jointVelocity = ecm_->Component<sim::components::JointVelocity>(joint_data.sim_joint);
            if (jointVelocity && jointVelocity->Data().size() > 0)
            {
                joint_data.velocity = jointVelocity->Data()[0];
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropArmHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        for (auto &[joint_name, joint_data] : joints_)
        {
            if (joint_data.sim_joint == sim::kNullEntity)
                continue;

            double motor_speed_command = 0.0;
            bool send_command = false;

            // Track previous commands to detect changes
            static std::map<std::string, double> last_position_cmd;
            static std::map<std::string, double> last_velocity_cmd;
            static std::map<std::string, double> last_effort_cmd;
            static std::map<std::string, std::chrono::steady_clock::time_point> last_cmd_time;

            auto now = std::chrono::steady_clock::now();
            const double cmd_threshold = 1e-4;

            // Initialize if needed
            if (last_position_cmd.find(joint_name) == last_position_cmd.end())
            {
                last_position_cmd[joint_name] = joint_data.position_command;
                last_velocity_cmd[joint_name] = joint_data.velocity_command;
                last_effort_cmd[joint_name] = joint_data.effort_command;
                last_cmd_time[joint_name] = now;
            }

            // PRIORITY 1: Position Control (MIT's primary objective - angle control)
            if (std::abs(joint_data.position_command - last_position_cmd[joint_name]) > cmd_threshold)
            {
                auto &pid = pid_controllers_[joint_name];

                double target_angle = joint_data.position_command; // Target angle (radians)
                double current_angle = joint_data.position;        // Current angle (radians)

                // Calculate angle error with proper wrapping
                double error = target_angle - current_angle;
                while (error > M_PI)
                    error -= 2.0 * M_PI;
                while (error < -M_PI)
                    error += 2.0 * M_PI;

                // PID calculation
                pid.integral += error * period.seconds();

                // Anti-windup with practical limits
                double integral_limit = 1.0;
                pid.integral = std::max(-integral_limit, std::min(integral_limit, pid.integral));

                double derivative = 0.0;
                if (period.seconds() > 1e-6)
                {
                    derivative = (error - pid.prev_error) / period.seconds();
                }

                // MIT PropArm Physics: Calculate required motor speed for angle control
                // Base thrust needed to counteract gravity component: mg*sin(θ)
                // For MIT model, this translates to motor speed needed
                double gravity_compensation = 250.0 * std::sin(target_angle);

                // PID output for motor speed
                double pid_output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

                // Total motor speed = gravity compensation + PID correction
                motor_speed_command = std::abs(gravity_compensation + pid_output);

                // Apply motor speed limits (thrust is always positive)
                motor_speed_command = std::max(pid.output_min, std::min(pid.output_max, motor_speed_command));

                pid.prev_error = error;
                send_command = true;
                last_position_cmd[joint_name] = joint_data.position_command;
                last_cmd_time[joint_name] = now;

                RCLCPP_INFO_THROTTLE(this->nh_->get_logger(), *this->nh_->get_clock(), 2000,
                                     "ANGLE CONTROL: target=%.1f°, current=%.1f°, error=%.1f°, motor=%.1f rad/s",
                                     target_angle * 180.0 / M_PI, current_angle * 180.0 / M_PI,
                                     error * 180.0 / M_PI, motor_speed_command);
            }
            // PRIORITY 2: Direct Velocity Control
            else if (std::abs(joint_data.velocity_command - last_velocity_cmd[joint_name]) > cmd_threshold)
            {
                // Direct motor speed command (always positive for thrust)
                motor_speed_command = std::abs(joint_data.velocity_command);
                motor_speed_command = std::max(0.0, std::min(785.0, motor_speed_command));

                send_command = true;
                last_velocity_cmd[joint_name] = joint_data.velocity_command;
                last_cmd_time[joint_name] = now;

                RCLCPP_INFO_THROTTLE(this->nh_->get_logger(), *this->nh_->get_clock(), 2000,
                                     "VELOCITY CONTROL: motor=%.1f rad/s", motor_speed_command);
            }
            // PRIORITY 3: Force/Effort Control (convert to motor speed)
            else if (std::abs(joint_data.effort_command - last_effort_cmd[joint_name]) > cmd_threshold)
            {
                // Convert force to motor speed using MIT model: F = k * ω²
                double force_magnitude = std::abs(joint_data.effort_command);
                if (force_magnitude > 1e-3)
                {
                    // F = motorConstant * ω²  =>  ω = sqrt(F / motorConstant)
                    motor_speed_command = std::sqrt(force_magnitude / model_params_.motorConstant);
                }
                else
                {
                    motor_speed_command = 0.0;
                }

                // Apply limits
                motor_speed_command = std::max(0.0, std::min(785.0, motor_speed_command));

                send_command = true;
                last_effort_cmd[joint_name] = joint_data.effort_command;
                last_cmd_time[joint_name] = now;

                RCLCPP_INFO_THROTTLE(this->nh_->get_logger(), *this->nh_->get_clock(), 2000,
                                     "FORCE CONTROL: force=%.1fN -> motor=%.1f rad/s",
                                     force_magnitude, motor_speed_command);
            }

            // Send command to Gazebo motor plugin
            if (send_command)
            {
                gz::msgs::Actuators actuators_msg;
                actuators_msg.add_velocity(motor_speed_command);

                bool success = actuators_pub_.Publish(actuators_msg);

                if (!success)
                {
                    RCLCPP_WARN_THROTTLE(this->nh_->get_logger(), *this->nh_->get_clock(), 5000,
                                         "Failed to publish motor command to %s", actuators_topic_.c_str());
                }
                else
                {
                    RCLCPP_DEBUG_THROTTLE(this->nh_->get_logger(), *this->nh_->get_clock(), 1000,
                                          "Motor command sent: %.1f rad/s", motor_speed_command);
                }
            }
            // Handle timeout - maintain position with gravity compensation
            else if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_time[joint_name]).count() > 3000)
            {
                // No commands for 3 seconds, provide basic gravity compensation
                double hold_speed = 200.0 * std::abs(std::sin(joint_data.position));

                gz::msgs::Actuators hold_msg;
                hold_msg.add_velocity(hold_speed);
                actuators_pub_.Publish(hold_msg);

                last_cmd_time[joint_name] = now;

                RCLCPP_DEBUG_THROTTLE(this->nh_->get_logger(), *this->nh_->get_clock(), 5000,
                                      "Timeout: applying gravity compensation %.1f rad/s", hold_speed);
            }
        }

        return hardware_interface::return_type::OK;
    }

    // Helper functions remain the same
    double PropArmHardware::mitToGazeboAngle(double mit_angle) const
    {
        return mit_angle; // No transformation needed with corrected URDF
    }

    double PropArmHardware::gazeboToMitAngle(double gazebo_angle) const
    {
        return gazebo_angle; // No transformation needed with corrected URDF
    }

    void PropArmHardware::updateElectroMechanicalModel(const std::string &joint_name, double motor_speed_cmd, double dt)
    {
        auto &joint = joints_[joint_name];

        // MIT electro-mechanical model simulation
        double omega_a = joint.velocity;
        double motor_thrust = model_params_.motorConstant * motor_speed_cmd * motor_speed_cmd; // F = k*ω²
        double motor_torque = motor_thrust * model_params_.momentConstant;

        // Friction and damping forces
        double friction_torque = model_params_.Kf * std::copysign(1.0, omega_a) * omega_a * omega_a;
        double damping_torque = model_params_.Ra * omega_a;

        // Angular acceleration
        double omega_a_dot = (motor_torque - damping_torque - friction_torque) / model_params_.Ja;

        // Integration
        joint.velocity += omega_a_dot * dt;
        joint.position += joint.velocity * dt;
        joint.effort = motor_torque;

        // Apply reasonable limits
        joint.position = std::fmod(joint.position + M_PI, 2.0 * M_PI) - M_PI;
        joint.velocity *= 0.998; // Light velocity damping
    }

    // Standard ros2_control interface methods implementation
    hardware_interface::CallbackReturn PropArmHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PropArmHardware::export_state_interfaces()
    {
        return std::move(state_interfaces_);
    }

    std::vector<hardware_interface::CommandInterface> PropArmHardware::export_command_interfaces()
    {
        return std::move(command_interfaces_);
    }

    hardware_interface::CallbackReturn PropArmHardware::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Activating MIT PropArm Hardware");

        // Reset PID controllers
        for (auto &[joint_name, pid] : pid_controllers_)
        {
            pid.prev_error = 0.0;
            pid.integral = 0.0;
        }

        // Initialize commands to safe defaults
        for (auto &[joint_name, joint_data] : joints_)
        {
            joint_data.position_command = 0.0; // Start horizontal
            joint_data.velocity_command = 0.0;
            joint_data.effort_command = 0.0;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Deactivating MIT PropArm Hardware");

        // Send stop command to motor
        gz::msgs::Actuators stop_msg;
        stop_msg.add_velocity(0.0);
        actuators_pub_.Publish(stop_msg);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace prop_arm_gazebo_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    prop_arm_gazebo_control::PropArmHardware,
    gz_ros2_control::GazeboSimSystemInterface)
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

        // Get robot namespace
        robot_namespace_ = "prop_arm";
        auto param_it = hardware_info.hardware_parameters.find("robot_namespace");
        if (param_it != hardware_info.hardware_parameters.end()) {
            robot_namespace_ = param_it->second;
        }

        // Initialize Gazebo transport
        gz_node_ = std::make_unique<gz::transport::Node>();
        actuators_topic_ = "/" + robot_namespace_ + "/command/motor_speed";
        actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Initialize joints
        for (const auto &joint_info : hardware_info.joints) {
            const std::string &joint_name = joint_info.name;

            auto it = enabled_joints_.find(joint_name);
            if (it == enabled_joints_.end()) {
                RCLCPP_WARN(this->nh_->get_logger(), "Joint '%s' not found", joint_name.c_str());
                continue;
            }

            // Initialize joint data
            joints_[joint_name] = JointData{};
            joints_[joint_name].sim_joint = it->second;
            
            // Initialize PID with corrected parameters
            pid_controllers_[joint_name] = PIDController{};
            pid_controllers_[joint_name].kp = 0.0;    // Disable PID position control
            pid_controllers_[joint_name].ki = 0.0;    // Let effort controller handle it
            pid_controllers_[joint_name].kd = 0.0;    // Avoid conflicts
            pid_controllers_[joint_name].output_min = 0.0;
            pid_controllers_[joint_name].output_max = 785.0;

            // Create Gazebo components
            sim::Entity joint_entity = it->second;
            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointPosition().TypeId())) {
                _ecm.CreateComponent(joint_entity, sim::components::JointPosition());
            }
            if (!_ecm.EntityHasComponentType(joint_entity, sim::components::JointVelocity().TypeId())) {
                _ecm.CreateComponent(joint_entity, sim::components::JointVelocity());
            }

            // Setup interfaces
            for (const auto &interface : joint_info.state_interfaces) {
                if (interface.name == hardware_interface::HW_IF_POSITION) {
                    state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, 
                                                 &joints_[joint_name].position);
                } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
                    state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, 
                                                 &joints_[joint_name].velocity);
                } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
                    state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, 
                                                 &joints_[joint_name].effort);
                }
            }

            for (const auto &interface : joint_info.command_interfaces) {
                if (interface.name == hardware_interface::HW_IF_POSITION) {
                    command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, 
                                                   &joints_[joint_name].position_command);
                } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
                    command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, 
                                                   &joints_[joint_name].velocity_command);
                } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
                    command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, 
                                                   &joints_[joint_name].effort_command);
                }
            }

            RCLCPP_INFO(this->nh_->get_logger(), "Joint initialized: %s", joint_name.c_str());
        }

        RCLCPP_INFO(this->nh_->get_logger(), "Hardware interface ready - %zu joints", joints_.size());
        return true;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
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

    hardware_interface::CallbackReturn PropArmHardware::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Activating PropArm Hardware");

        // Initialize all commands to zero
        for (auto &[joint_name, joint_data] : joints_) {
            joint_data.position = 0.0;
            joint_data.velocity = 0.0;
            joint_data.effort = 0.0;
            joint_data.velocity_command = 0.0;
            joint_data.position_command = 0.0;
            joint_data.effort_command = 0.0;
            joint_data.last_effort_command = 0.0;
            joint_data.last_velocity_command = 0.0;
            joint_data.last_position_command = 0.0;
        }

        // Send initial zero command
        gz::msgs::Actuators actuators_msg;
        actuators_msg.add_velocity(0.0);
        actuators_pub_.Publish(actuators_msg);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Deactivating PropArm Hardware");

        // Stop motor
        gz::msgs::Actuators actuators_msg;
        actuators_msg.add_velocity(0.0);
        actuators_pub_.Publish(actuators_msg);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    double PropArmHardware::mitToGazeboAngle(double mit_angle) const
    {
        return mit_angle;  // No conversion needed with corrected URDF
    }

    double PropArmHardware::gazeboToMitAngle(double gazebo_angle) const
    {
        return gazebo_angle;  // No conversion needed with corrected URDF
    }

    void PropArmHardware::updateElectroMechanicalModel(const std::string &joint_name,
                                                       double motor_command, double dt)
    {
        auto &joint = joints_[joint_name];

        // Simplified model: motor command directly controls thrust
        motor_command = std::max(0.0, motor_command);  // Positive only

        // Convert motor speed to force (F = k * ω²)
        double k_motor = 0.008;
        double thrust_force = k_motor * motor_command * motor_command;

        // Update effort for state interface
        joint.effort = thrust_force;
    }

    hardware_interface::return_type PropArmHardware::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Read joint states from Gazebo
        for (auto &[joint_name, joint_data] : joints_) {
            if (joint_data.sim_joint == sim::kNullEntity) continue;

            // Get position
            const auto *jointPositions = ecm_->Component<sim::components::JointPosition>(joint_data.sim_joint);
            if (jointPositions && jointPositions->Data().size() > 0) {
                joint_data.position = gazeboToMitAngle(jointPositions->Data()[0]);
            }

            // Get velocity
            const auto *jointVelocity = ecm_->Component<sim::components::JointVelocity>(joint_data.sim_joint);
            if (jointVelocity && jointVelocity->Data().size() > 0) {
                joint_data.velocity = jointVelocity->Data()[0];
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropArmHardware::write(
        const rclcpp::Time &, const rclcpp::Duration &period)
    {
        for (auto &[joint_name, joint_data] : joints_) {
            if (joint_data.sim_joint == sim::kNullEntity) continue;

            double motor_command = 0.0;
            bool command_received = false;

            // Check for command changes
            bool effort_changed = (std::abs(joint_data.effort_command - joint_data.last_effort_command) > 1e-6);
            bool velocity_changed = (std::abs(joint_data.velocity_command - joint_data.last_velocity_command) > 1e-6);

            // Priority: Effort > Velocity (SIMPLIFIED - no position control here)
            if (std::abs(joint_data.effort_command) > 1e-6 || effort_changed) {
                // Force mode - convert to motor speed
                double force_command = std::max(0.0, joint_data.effort_command);
                
                if (force_command > 0.001) {
                    double k_motor = 0.008;
                    motor_command = std::sqrt(force_command / k_motor);
                }
                
                command_received = true;
                joint_data.last_effort_command = joint_data.effort_command;

                RCLCPP_DEBUG(rclcpp::get_logger("PropArmHardware"),
                           "Effort mode: %.2fN -> %.2f rad/s", force_command, motor_command);
                           
            } else if (std::abs(joint_data.velocity_command) > 1e-6 || velocity_changed) {
                // Direct velocity mode
                motor_command = std::max(0.0, joint_data.velocity_command);
                command_received = true;
                joint_data.last_velocity_command = joint_data.velocity_command;

                RCLCPP_DEBUG(rclcpp::get_logger("PropArmHardware"),
                           "Velocity mode: %.2f rad/s", motor_command);
            }

            // Send command to motor
            if (command_received) {
                motor_command = std::max(0.0, std::min(785.0, motor_command));
                
                gz::msgs::Actuators actuators_msg;
                actuators_msg.add_velocity(motor_command);
                bool success = actuators_pub_.Publish(actuators_msg);
                
                if (!success) {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PropArmHardware"), 
                                         *nh_->get_clock(), 1000,
                                         "Failed to publish motor command");
                }

                // Update model
                updateElectroMechanicalModel(joint_name, motor_command, period.seconds());
            } else {
                // No command - send zero
                gz::msgs::Actuators actuators_msg;
                actuators_msg.add_velocity(0.0);
                actuators_pub_.Publish(actuators_msg);
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace prop_arm_gazebo_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    prop_arm_gazebo_control::PropArmHardware,
    gz_ros2_control::GazeboSimSystemInterface)
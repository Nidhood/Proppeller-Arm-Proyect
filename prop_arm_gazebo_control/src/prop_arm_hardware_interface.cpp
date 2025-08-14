#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

namespace prop_arm_gazebo_control
{

    bool PropArmHardware::initSim(rclcpp::Node::SharedPtr &model_nh,
                                  std::map<std::string, sim::Entity> &joints,
                                  const hardware_interface::HardwareInfo &hardware_info,
                                  sim::EntityComponentManager &ecm,
                                  unsigned int /*update_rate*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Initializing PropArm Hardware Interface");

        nh_ = model_nh;
        ecm_ = &ecm;
        enabled_joints_ = joints;

        // Parameters from URDF
        if (auto it = hardware_info.hardware_parameters.find("robot_namespace");
            it != hardware_info.hardware_parameters.end())
            robot_namespace_ = it->second;

        if (auto it = hardware_info.hardware_parameters.find("actuator_index");
            it != hardware_info.hardware_parameters.end())
            actuator_index_ = std::max(0, std::stoi(it->second));

        if (auto it = hardware_info.hardware_parameters.find("max_rot_velocity_radps");
            it != hardware_info.hardware_parameters.end())
            max_rot_vel_ = std::max(0.0, std::stod(it->second));

        // Topic: /<ns>/command/motor_speed
        actuators_topic_ = "/" + robot_namespace_ + "/command/motor_speed";
        gz_node_ = std::make_unique<gz::transport::Node>();
        actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);
        RCLCPP_INFO(nh_->get_logger(), "Actuators topic: %s (actuator_index=%d, vmax=%.2f rad/s)",
                    actuators_topic_.c_str(), actuator_index_, max_rot_vel_);

        // Publisher initialization:
        if (auto it = hardware_info.hardware_parameters.find("tau_up");
            it != hardware_info.hardware_parameters.end())
            tau_up_ = std::max(1e-6, std::stod(it->second));
        if (auto it = hardware_info.hardware_parameters.find("tau_down");
            it != hardware_info.hardware_parameters.end())
            tau_down_ = std::max(1e-6, std::stod(it->second));

        motor_speed_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            "/prop_arm/motor_speed_est", rclcpp::QoS(10).best_effort());

        // rm angle publisher (in degrees)
        arm_angle_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            "/prop_arm/arm_angle_deg", rclcpp::QoS(10).best_effort());

        // Build interfaces
        state_interfaces_.clear();
        command_interfaces_.clear();
        state_interfaces_.reserve(hardware_info.joints.size() * 2);
        command_interfaces_.reserve(hardware_info.joints.size());

        for (const auto &ji : hardware_info.joints)
        {
            const std::string &name = ji.name;

            auto it = enabled_joints_.find(name);
            if (it == enabled_joints_.end())
            {
                RCLCPP_WARN(nh_->get_logger(), "Joint '%s' not found in sim", name.c_str());
                continue;
            }

            auto &jd = joints_[name]; // creates default JointData
            jd.sim_joint = it->second;

            // Ensure position/velocity components exist
            if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointPosition().TypeId()))
                ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointPosition());
            if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointVelocity().TypeId()))
                ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointVelocity());

            // State interfaces
            state_interfaces_.emplace_back(name, hardware_interface::HW_IF_POSITION, &jd.position);
            state_interfaces_.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &jd.velocity);

            // Velocity command interface only
            command_interfaces_.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_command);

            RCLCPP_INFO(nh_->get_logger(), "Joint ready: %s", name.c_str());
        }

        RCLCPP_INFO(nh_->get_logger(), "Hardware interface ready - %zu joints", joints_.size());
        return true;
    }

    hardware_interface::CallbackReturn
    PropArmHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        return (hardware_interface::SystemInterface::on_init(info) ==
                hardware_interface::CallbackReturn::SUCCESS)
                   ? hardware_interface::CallbackReturn::SUCCESS
                   : hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<hardware_interface::StateInterface> PropArmHardware::export_state_interfaces()
    {
        return std::move(state_interfaces_); // exported once
    }

    std::vector<hardware_interface::CommandInterface> PropArmHardware::export_command_interfaces()
    {
        return std::move(command_interfaces_); // exported once
    }

    hardware_interface::CallbackReturn
    PropArmHardware::on_activate(const rclcpp_lifecycle::State &)
    {
        for (auto &kv : joints_)
        {
            kv.second.position = 0.0;
            kv.second.velocity = 0.0;
            kv.second.velocity_command = 0.0;
        }

        // Publish 0 rad/s at the configured actuator index
        gz::msgs::Actuators msg;
        msg.mutable_velocity()->Resize(actuator_index_ + 1, 0.0);
        actuators_pub_.Publish(msg);

        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Activated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    PropArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
    {
        gz::msgs::Actuators msg;
        msg.mutable_velocity()->Resize(actuator_index_ + 1, 0.0);
        actuators_pub_.Publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Deactivated (motor stopped).");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    PropArmHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for (auto &kv : joints_)
        {
            auto &jd = kv.second;
            if (jd.sim_joint == sim::kNullEntity)
                continue;

            const auto *pos = ecm_->Component<gz::sim::components::JointPosition>(jd.sim_joint);
            if (pos && !pos->Data().empty())
                jd.position = pos->Data()[0];

            const auto *vel = ecm_->Component<gz::sim::components::JointVelocity>(jd.sim_joint);
            if (vel && !vel->Data().empty())
                jd.velocity = vel->Data()[0];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    PropArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        double dt = period.seconds();
        dt = std::max(0.0, dt);

        double motor_cmd = 0.0;
        if (!joints_.empty())
        {
            const auto &jd = joints_.begin()->second;
            motor_cmd = std::clamp(jd.velocity_command, 0.0, max_rot_vel_);
        }

        // First-order response to command, matching plugin's idea:
        const double tau = (motor_cmd >= motor_speed_est_) ? tau_up_ : tau_down_;
        const double alpha = (tau > 0.0) ? dt / (tau + dt) : 1.0;
        motor_speed_est_ += alpha * (motor_cmd - motor_speed_est_);
        motor_speed_est_ = std::clamp(motor_speed_est_, 0.0, max_rot_vel_);

        // Publish motor speed to ROS for your VEmfPublisher
        std_msgs::msg::Float64 spd;
        spd.data = motor_speed_est_;
        motor_speed_pub_->publish(spd);

        // Publish arm angle in degrees
        auto arm_joint_it = joints_.find("arm_link_joint");
        if (arm_joint_it != joints_.end())
        {
            std_msgs::msg::Float64 arm_angle_msg;
            // Convert from radians to degrees
            arm_angle_msg.data = arm_joint_it->second.position * 180.0 / M_PI;
            arm_angle_pub_->publish(arm_angle_msg);
        }

        // Still send the actual command to Gazebo
        gz::msgs::Actuators msg;
        msg.mutable_velocity()->Resize(actuator_index_ + 1, 0.0);
        msg.set_velocity(actuator_index_, motor_cmd);
        if (!actuators_pub_.Publish(msg))
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PropArmHardware"), *nh_->get_clock(), 1000,
                                 "Failed to publish motor command");
        }
        return hardware_interface::return_type::OK;
    }

} // namespace prop_arm_gazebo_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(prop_arm_gazebo_control::PropArmHardware,
                       gz_ros2_control::GazeboSimSystemInterface)
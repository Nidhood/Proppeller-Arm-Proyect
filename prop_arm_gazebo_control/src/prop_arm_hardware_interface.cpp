#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <type_traits>
#include <algorithm>
#include <string>
namespace prop_arm_gazebo_control
{

    // ========================================================================
    //                            INITIALIZATION                             //
    // ========================================================================

    // Hardware interface initialization in Gazebo simulation:
    bool PropArmHardware::initSim(rclcpp::Node::SharedPtr &model_nh,
                                  std::map<std::string, sim::Entity> &joints,
                                  const hardware_interface::HardwareInfo &hardware_info,
                                  sim::EntityComponentManager &ecm,
                                  unsigned int /*update_rate*/)
    {
        // Node, Gazebo transport, and enabled joints handling:
        nh_ = model_nh;
        ecm_ = &ecm;
        enabled_joints_ = joints;

        // Load parameters from XACRO
        loadParameters(hardware_info);

        // Initialize motor model
        motor_model_ = prop_arm_characterization::MotorSpeedModel{
            Kw_, tau_w_, Ts_, pwm_ref_us_, 0.0};

        // =======================================
        // Setup communication between Gazebo and ROS2
        // =======================================

        // Setup Gazebo transport:
        gz_node_ = std::make_unique<gz::transport::Node>();
        actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

        // Setup ROS2 publishers:
        motor_speed_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            nsTopic(robot_namespace_, "motor_vel_rad"),
            rclcpp::QoS(100).best_effort());

        arm_angle_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            nsTopic(robot_namespace_, "angle_rad"),
            rclcpp::QoS(50).best_effort());

        // PWM feedback publisher (UInt16)
        vpwm_pub_ = nh_->create_publisher<std_msgs::msg::UInt16>(
            nsTopic(robot_namespace_, "esc/pwm_us_fb"),
            rclcpp::QoS(400).best_effort());

        // Setup ROS2 subscribers: PWM command as UInt16
        pwm_cmd_sub_ = nh_->create_subscription<std_msgs::msg::UInt16>(
            nsTopic(robot_namespace_, "esc/pwm_us"),
            rclcpp::QoS(400).best_effort(),
            [this](const std_msgs::msg::UInt16::SharedPtr msg)
            {
                const auto raw_pwm = msg->data;
                const int clamped = std::clamp(
                    static_cast<int>(raw_pwm),
                    static_cast<int>(pwm_min_us_),
                    static_cast<int>(pwm_max_us_));
                current_pwm_us_ = static_cast<std::uint16_t>(clamped);
            });

        // Setup ros2_control interfaces
        setupControlInterfaces(hardware_info, ecm);

        return true;
    }

    // ========================================================================
    // LIFECYCLE MANAGEMENT
    // ========================================================================

    // Hardware interface on_init callback.
    hardware_interface::CallbackReturn
    PropArmHardware::on_init(const hardware_interface::HardwareInfo & /*info*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Hardware interface on_activate callback.
    hardware_interface::CallbackReturn
    PropArmHardware::on_activate(const rclcpp_lifecycle::State &)
    {
        // Reset joint states:
        auto &jd = joints_[joint_name_];
        jd.position_output = 0.0;
        jd.velocity_output = 0.0;

        // Stop motor:
        gz::msgs::Actuators msg;
        actuators_pub_.Publish(msg);

        // Reset motor model:
        motor_model_.reset();
        current_pwm_us_ = pwm_ref_us_;

        RCLCPP_INFO(nh_->get_logger(), "PropArmHardware activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Hardware interface on_deactivate callback.
    hardware_interface::CallbackReturn
    PropArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Stop motor
        gz::msgs::Actuators msg;
        actuators_pub_.Publish(msg);

        RCLCPP_INFO(nh_->get_logger(), "PropArmHardware deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ========================================================================
    // INTERFACE EXPORTS
    // ========================================================================

    // Export state interfaces to ros2_control.
    std::vector<hardware_interface::StateInterface>
    PropArmHardware::export_state_interfaces()
    {
        return std::move(state_interfaces_);
    }

    // Export command interfaces to ros2_control.
    std::vector<hardware_interface::CommandInterface>
    PropArmHardware::export_command_interfaces()
    {
        return std::move(command_interfaces_);
    }

    // ========================================================================
    // READ/WRITE CYCLE
    // ========================================================================

    // Read joint states from Gazebo simulation.
    hardware_interface::return_type
    PropArmHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        auto &jd = joints_[joint_name_];

        // Read joint position
        const auto *pos = ecm_->Component<gz::sim::components::JointPosition>(jd.sim_joint);
        if (pos && !pos->Data().empty())
            jd.position_output = pos->Data()[0];

        // Read joint velocity
        const auto *vel = ecm_->Component<gz::sim::components::JointVelocity>(jd.sim_joint);
        if (vel && !vel->Data().empty())
            jd.velocity_output = vel->Data()[0];

        return hardware_interface::return_type::OK;
    }

    // Write commands to Gazebo and publish telemetry.
    hardware_interface::return_type
    PropArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Update motor model with PWM command
        motor_model_.update(current_pwm_us_);
        double motor_speed = motor_model_.getSpeedRadSec();

        // Send command to Gazebo
        publishToGazebo(motor_speed);

        // Publish telemetry
        publishMotorTelemetry(motor_speed);
        publishJointTelemetry();

        return hardware_interface::return_type::OK;
    }

    // ========================================================================
    // SETUP HELPERS
    // ========================================================================

    // Load parameters from hardware info.
    void prop_arm_gazebo_control::PropArmHardware::loadParameters(
    const hardware_interface::HardwareInfo &hardware_info)
    {
        // Helper genérico para leer parámetros desde hardware_info.hardware_parameters
        auto getParam = [&](const std::string &name, auto &variable, auto default_val)
        {
            using T = std::decay_t<decltype(variable)>;

            auto it = hardware_info.hardware_parameters.find(name);
            if (it == hardware_info.hardware_parameters.end())
            {
                // Si no existe el parámetro, usar el valor por defecto
                if constexpr (std::is_same_v<T, std::string>)
                {
                    variable = default_val;
                }
                else
                {
                    variable = static_cast<T>(default_val);
                }
                return;
            }

            const std::string &val_str = it->second;

            if constexpr (std::is_same_v<T, std::string>)
            {
                // Parámetro tipo string
                variable = val_str;
            }
            else if constexpr (std::is_same_v<T, int>)
            {
                // Parámetro tipo int
                variable = static_cast<int>(std::stoi(val_str));
            }
            else if constexpr (std::is_integral_v<T>)
            {
                // Cualquier entero (incluye std::uint16_t, std::uint32_t, etc.)
                long long v = std::stoll(val_str);
                variable = static_cast<T>(v);
            }
            else if constexpr (std::is_floating_point_v<T>)
            {
                // Parámetro en coma flotante: aplicar mínimo con default_val
                double v = std::stod(val_str);
                double def = static_cast<double>(default_val);
                variable = static_cast<T>(std::max(def, v));
            }
            else
            {
                static_assert(std::is_arithmetic_v<T>,
                            "Tipo de parámetro no soportado en getParam");
            }
    };

    // Strings
    getParam("robot_namespace", robot_namespace_, std::string("prop_arm"));
    getParam("actuators_topic", actuators_topic_, std::string("/prop_arm/command/motor_speed"));
    getParam("joint_name",      joint_name_,      std::string("arm_link_joint"));

    // Parámetros físicos / modelo
    getParam("prop_radius_m", prop_radius_m_, 0.10); // [m]
    getParam("Kw",            Kw_,           0.0);   // [rad/s por unidad de entrada]
    getParam("tau_w",         tau_w_,        0.0);   // [s]
    getParam("Ts",            Ts_,           0.01);  // [s]

    // PWM (UInt16) en microsegundos
    getParam("pwm_ref_us", pwm_ref_us_, static_cast<std::uint16_t>(1500));
    getParam("pwm_min_us", pwm_min_us_, static_cast<std::uint16_t>(1000));
    getParam("pwm_max_us", pwm_max_us_, static_cast<std::uint16_t>(2000));

    // Índice del actuador en el mensaje Actuators
    getParam("actuator_index", actuator_index_, 0);
}


    // Setup ros2_control interfaces based on hardware info.
    void PropArmHardware::setupControlInterfaces(
        const hardware_interface::HardwareInfo &hardware_info,
        sim::EntityComponentManager &ecm)
    {
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

            auto &jd = joints_[name];
            jd.sim_joint = it->second;

            // Create Gazebo components
            if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointPosition().TypeId()))
                ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointPosition());

            if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointVelocity().TypeId()))
                ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointVelocity());

            // Export interfaces
            state_interfaces_.emplace_back(name, hardware_interface::HW_IF_POSITION, &jd.position_output);
            state_interfaces_.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_output);
        }
    }

    // ========================================================================
    // PUBLISHING HELPERS
    // ========================================================================

    // Publish motor speed command to Gazebo.
    void PropArmHardware::publishToGazebo(double motor_speed)
    {
        gz::msgs::Actuators msg;
        msg.add_velocity(motor_speed);

        if (!actuators_pub_.Publish(msg))
        {
            RCLCPP_WARN_THROTTLE(
                nh_->get_logger(), *nh_->get_clock(), 1000,
                "Failed to publish motor command to Gazebo");
        }
    }

    // Publish motor telemetry to ROS2 topics.
    void PropArmHardware::publishMotorTelemetry(double motor_speed)
    {
        // Motor speed [rad/s]
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = motor_speed;
        motor_speed_pub_->publish(speed_msg);

        // PWM feedback [us] as UInt16
        std_msgs::msg::UInt16 pwm_msg;
        pwm_msg.data = current_pwm_us_;
        vpwm_pub_->publish(pwm_msg);
    }

    // Publish joint telemetry to ROS2 topics.
    void PropArmHardware::publishJointTelemetry()
    {
        auto it = joints_.find(joint_name_);
        if (it != joints_.end())
        {
            std_msgs::msg::Float64 angle_msg;
            angle_msg.data = it->second.position_output;
            arm_angle_pub_->publish(angle_msg);
        }
    }

} // namespace prop_arm_gazebo_control

PLUGINLIB_EXPORT_CLASS(prop_arm_gazebo_control::PropArmHardware,
                       gz_ros2_control::GazeboSimSystemInterface)
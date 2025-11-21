#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <prop_arm_characterization/motor_speed_model.hpp>
#include <type_traits>
#include <algorithm>
#include <string>
#include <utility>

namespace prop_arm_gazebo_control
{

bool PropArmHardware::initSim(rclcpp::Node::SharedPtr &model_nh,
                              std::map<std::string, sim::Entity> &joints,
                              const hardware_interface::HardwareInfo &hardware_info,
                              sim::EntityComponentManager &ecm,
                              unsigned int /*update_rate*/)
{
    nh_ = model_nh;
    ecm_ = &ecm;
    enabled_joints_ = joints;

    loadParameters(hardware_info);

    // Inicializar modelo de velocidad del motor (5 parámetros: kw, tau_w, Ts, pwm_ref, w0)
    motor_model_ = prop_arm_characterization::MotorSpeedModel(
                       Kw_,        // kw
                       tau_w_,     // tau_w
                       Ts_,        // Ts
                       pwm_ref_us_,// pwm_ref_us
                       0.0         // w0
                   );

    gz_node_ = std::make_unique<gz::transport::Node>();
    actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

    motor_speed_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
                           nsTopic(robot_namespace_, "motor_vel_rad"),
                           rclcpp::QoS(100).best_effort());

    arm_angle_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
                         nsTopic(robot_namespace_, "angle_rad"),
                         rclcpp::QoS(50).best_effort());

    vpwm_pub_ = nh_->create_publisher<std_msgs::msg::UInt16>(
                    nsTopic(robot_namespace_, "esc/pwm_us_fb"),
                    rclcpp::QoS(400).best_effort());

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

    setupControlInterfaces(hardware_info, ecm);

    return true;
}

// ========================================================================
// LIFECYCLE MANAGEMENT
// ========================================================================

hardware_interface::CallbackReturn
PropArmHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    (void)info;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PropArmHardware::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(nh_->get_logger(), "PropArmHardware configured");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PropArmHardware::on_activate(const rclcpp_lifecycle::State &)
{
    auto &jd = joints_[joint_name_];
    jd.position_output = 0.0;
    jd.velocity_output = 0.0;

    gz::msgs::Actuators msg;
    actuators_pub_.Publish(msg);

    motor_model_.reset();
    pwm_delay_line_.clear();
    current_pwm_us_ = pwm_ref_us_;

    RCLCPP_INFO(nh_->get_logger(), "PropArmHardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PropArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    gz::msgs::Actuators msg;
    actuators_pub_.Publish(msg);

    RCLCPP_INFO(nh_->get_logger(), "PropArmHardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ========================================================================
// EXPORT INTERFACES
// ========================================================================

std::vector<hardware_interface::StateInterface>
PropArmHardware::export_state_interfaces()
{
    // Command/StateInterface son move-only en ros2_control -> devolver por move
    return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
PropArmHardware::export_command_interfaces()
{
    return std::move(command_interfaces_);
}

// ========================================================================
// READ/WRITE
// ========================================================================

hardware_interface::return_type
PropArmHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto &item : joints_)
    {
        const auto &name = item.first;
        auto &jd = item.second;

        auto posComp = ecm_->Component<gz::sim::components::JointPosition>(jd.sim_joint);
        auto velComp = ecm_->Component<gz::sim::components::JointVelocity>(jd.sim_joint);

        if (posComp && !posComp->Data().empty())
        {
            jd.position_output = posComp->Data().front();
        }
        if (velComp && !velComp->Data().empty())
        {
            jd.velocity_output = velComp->Data().front();
        }

        (void)name;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PropArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (current_pwm_us_ < pwm_min_us_)
        current_pwm_us_ = pwm_min_us_;
    else if (current_pwm_us_ > pwm_max_us_)
        current_pwm_us_ = pwm_max_us_;

    pwm_delay_line_.push_back(current_pwm_us_);

    std::uint16_t pwm_delayed = pwm_ref_us_;
    if (static_cast<int>(pwm_delay_line_.size()) > delay_steps_)
    {
        pwm_delayed = pwm_delay_line_.front();
        pwm_delay_line_.pop_front();
    }

    motor_model_.update(pwm_delayed);
    double motor_speed = motor_model_.getSpeedRadSec();
    double motor_speed_cmd = motor_cmd_scale_ * motor_speed;

    publishToGazebo(motor_speed_cmd);

    publishMotorTelemetry(motor_speed);
    publishJointTelemetry();

    return hardware_interface::return_type::OK;
}

// ========================================================================
// SETUP HELPERS
// ========================================================================

void PropArmHardware::loadParameters(
    const hardware_interface::HardwareInfo &hardware_info)
{
    auto getParam = [&](const std::string &name, auto &variable, auto default_val)
    {
        using T = std::decay_t<decltype(variable)>;

        auto it = hardware_info.hardware_parameters.find(name);
        if (it == hardware_info.hardware_parameters.end())
        {
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
            variable = val_str;
        }
        else if constexpr (std::is_same_v<T, int>)
        {
            int v = std::stoi(val_str);
            variable = static_cast<int>(v);
        }
        else if constexpr (std::is_integral_v<T>)
        {
            long long v = std::stoll(val_str);
            variable = static_cast<T>(v);
        }
        else if constexpr (std::is_floating_point_v<T>)
        {
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

    getParam("robot_namespace", robot_namespace_, std::string("prop_arm"));
    getParam("actuators_topic", actuators_topic_, std::string("/prop_arm/command/motor_speed"));
    getParam("joint_name",      joint_name_,      std::string("arm_link_joint"));

    getParam("prop_radius_m", prop_radius_m_, 0.10);
    getParam("Kw",            Kw_,           0.0);
    getParam("tau_w",         tau_w_,        0.0);
    getParam("L_w",           L_w_,          0.0);
    getParam("Ts",            Ts_,           0.01);

    getParam("pwm_ref_us", pwm_ref_us_, static_cast<std::uint16_t>(1500));
    getParam("pwm_min_us", pwm_min_us_, static_cast<std::uint16_t>(1000));
    getParam("pwm_max_us", pwm_max_us_, static_cast<std::uint16_t>(2000));

    getParam("motor_cmd_scale", motor_cmd_scale_, 1.0);

    if (Ts_ > 0.0)
    {
        delay_steps_ = static_cast<int>(std::round(L_w_ / Ts_));
        if (delay_steps_ < 0)
            delay_steps_ = 0;
    }
    else
    {
        delay_steps_ = 0;
    }
    pwm_delay_line_.clear();

    getParam("actuator_index", actuator_index_, 0);
}

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
            RCLCPP_WARN(nh_->get_logger(),
                        "Joint '%s' is not enabled in Gazebo", name.c_str());
            continue;
        }

        auto &jd = joints_[name];
        jd.sim_joint = it->second;

        if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointPosition().TypeId()))
            ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointPosition());

        if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointVelocity().TypeId()))
            ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointVelocity());

        state_interfaces_.emplace_back(name, hardware_interface::HW_IF_POSITION, &jd.position_output);
        state_interfaces_.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_output);

        bool hasEffortCommand = false;
        for (const auto &ci : ji.command_interfaces)
        {
            if (ci.name == hardware_interface::HW_IF_EFFORT)
            {
                hasEffortCommand = true;
                break;
            }
        }

        if (hasEffortCommand)
        {
            command_interfaces_.emplace_back(name, hardware_interface::HW_IF_EFFORT, &jd.command_effort);
        }
        else
        {
            RCLCPP_WARN(nh_->get_logger(),
                        "Joint '%s' has no 'effort' command interface defined; skipping command export",
                        name.c_str());
        }
    }
}

// ========================================================================
// PUBLISH HELPERS
// ========================================================================

std::string PropArmHardware::nsTopic(const std::string &ns, const std::string &topic) const
{
    if (ns.empty())
    {
        return topic;
    }
    if (ns.back() == '/')
    {
        return ns + topic;
    }
    return ns + "/" + topic;
}

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

void PropArmHardware::publishMotorTelemetry(double motor_speed)
{
    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = motor_speed;
    motor_speed_pub_->publish(speed_msg);

    std_msgs::msg::UInt16 pwm_msg;
    pwm_msg.data = current_pwm_us_;
    vpwm_pub_->publish(pwm_msg);
}

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

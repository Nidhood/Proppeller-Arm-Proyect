#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <gz/msgs/actuators.pb.h>
#include <gz/transport/Node.hh>

#include "pluginlib/class_list_macros.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <type_traits>
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

    // Build motor model (ARX(2,2) + delay inside the model)
    motor_model_ = prop_arm_characterization::MotorSpeedModel(
                       Ts_,
                       L_w_,
                       c1_, c2_, d1_, d2_,
                       pwm_min_us_,
                       pwm_max_us_,
                       pwm_spin_min_us_,
                       motor_cmd_scale_,
                       0.0,              // w0
                       prop_radius_m_     // prop_radius_m (kept for compatibility)
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
        std::scoped_lock lk(motor_mtx_);
        current_pwm_us_ = static_cast<std::uint16_t>(clamped);
    });

    setupControlInterfaces(hardware_info, ecm);

    declareRuntimeParams_();
    param_cb_handle_ = nh_->add_on_set_parameters_callback(
                           [this](const std::vector<rclcpp::Parameter> &params)
    {
        return this->onParams_(params);
    });

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

    {
        std::scoped_lock lk(motor_mtx_);
        motor_model_.reset(0.0);
        current_pwm_us_ = pwm_ref_us_;
    }

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
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PropArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    double motor_speed = 0.0;
    double motor_speed_cmd = 0.0;
    std::uint16_t pwm_snapshot = pwm_ref_us_;

    {
        std::scoped_lock lk(motor_mtx_);

        // Clamp current PWM against (possibly runtime-updated) limits
        const int clamped = std::clamp(
                                static_cast<int>(current_pwm_us_),
                                static_cast<int>(pwm_min_us_),
                                static_cast<int>(pwm_max_us_));
        current_pwm_us_ = static_cast<std::uint16_t>(clamped);

        pwm_snapshot = current_pwm_us_;

        // NOTE: delay L_w is implemented inside motor_model_ now (ARX + delay).
        motor_speed = motor_model_.update(current_pwm_us_);
        motor_speed_cmd = motor_speed;
    }

    // Joint velocity for friction/loss term
    double qd = 0.0;
    if (auto it = joints_.find(joint_name_); it != joints_.end())
    {
        qd = it->second.velocity_output;
    }

    const double visc = rt_viscous_arm_.load(std::memory_order_relaxed);
    const double coul = rt_coulomb_arm_.load(std::memory_order_relaxed);
    const double thrust_k = rt_thrust_k_.load(std::memory_order_relaxed);

    const double loss = visc * std::abs(qd) + coul * std::abs(signNoZero_(qd));
    motor_speed_cmd = thrust_k * motor_speed_cmd - loss;

    publishToGazebo(motor_speed_cmd);
    publishMotorTelemetry(motor_speed, pwm_snapshot);
    publishJointTelemetry();

    return hardware_interface::return_type::OK;
}

// ========================================================================
// SETUP HELPERS
// ========================================================================

void PropArmHardware::loadParameters(const hardware_interface::HardwareInfo &hardware_info)
{
    auto getParam = [&](const std::string &name, auto &variable, auto default_val)
    {
        using T = std::decay_t<decltype(variable)>;

        auto it = hardware_info.hardware_parameters.find(name);
        if (it == hardware_info.hardware_parameters.end())
        {
            if constexpr (std::is_same_v<T, std::string>)
                variable = default_val;
            else
                variable = static_cast<T>(default_val);
            return;
        }

        const std::string &val_str = it->second;

        if constexpr (std::is_same_v<T, std::string>)
        {
            variable = val_str;
        }
        else if constexpr (std::is_same_v<T, int>)
        {
            variable = static_cast<int>(std::stoi(val_str));
        }
        else if constexpr (std::is_integral_v<T>)
        {
            variable = static_cast<T>(std::stoll(val_str));
        }
        else if constexpr (std::is_floating_point_v<T>)
        {
            const double v = std::stod(val_str);
            const double def = static_cast<double>(default_val);
            variable = static_cast<T>(std::max(def, v));
        }
        else
        {
            static_assert(std::is_arithmetic_v<T>,
                          "Unsupported parameter type in getParam");
        }
    };

    getParam("robot_namespace", robot_namespace_, std::string("prop_arm"));
    getParam("actuators_topic", actuators_topic_, std::string("/prop_arm/command/motor_speed"));
    getParam("joint_name",      joint_name_,      std::string("arm_link_joint"));

    getParam("prop_radius_m", prop_radius_m_, 0.10);

    // Legacy (optional)
    getParam("Kw",            Kw_,            0.0);
    getParam("tau_w",         tau_w_,         0.0);

    // Preferred ARX parameters
    getParam("c1", c1_, 1.9823);
    getParam("c2", c2_, -0.9825);
    getParam("d1", d1_, 1.12e-4);
    getParam("d2", d2_, 1.11e-4);

    getParam("L_w",           L_w_,           0.10);
    getParam("Ts",            Ts_,            0.01);

    getParam("pwm_ref_us", pwm_ref_us_, static_cast<std::uint16_t>(1000));
    getParam("pwm_min_us", pwm_min_us_, static_cast<std::uint16_t>(1000));
    getParam("pwm_max_us", pwm_max_us_, static_cast<std::uint16_t>(2000));

    // IMPORTANT: this must be passed from xacro with param name="pwm_spin_min_us"
    getParam("pwm_spin_min_us", pwm_spin_min_us_, static_cast<std::uint16_t>(0));

    getParam("motor_cmd_scale", motor_cmd_scale_, 1.0);
    getParam("actuator_index",  actuator_index_,  0);

    // Sanity clamp for pwm_spin_min_us_
    if (pwm_spin_min_us_ < pwm_min_us_) pwm_spin_min_us_ = pwm_min_us_;
    if (pwm_spin_min_us_ > pwm_max_us_) pwm_spin_min_us_ = pwm_max_us_;

    // Optional runtime params init
    if (auto itv = hardware_info.hardware_parameters.find("runtime_viscous_arm");
            itv != hardware_info.hardware_parameters.end())
        rt_viscous_arm_.store(std::stod(itv->second));

    if (auto itc = hardware_info.hardware_parameters.find("runtime_coulomb_arm");
            itc != hardware_info.hardware_parameters.end())
        rt_coulomb_arm_.store(std::stod(itc->second));

    if (auto itk = hardware_info.hardware_parameters.find("runtime_thrust_k");
            itk != hardware_info.hardware_parameters.end())
        rt_thrust_k_.store(std::stod(itk->second));
}

void PropArmHardware::setupControlInterfaces(const hardware_interface::HardwareInfo &hardware_info,
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
            RCLCPP_WARN(nh_->get_logger(), "Joint '%s' is not enabled in Gazebo", name.c_str());
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
                        "Joint '%s' has no 'effort' command interface; skipping command export",
                        name.c_str());
        }
    }
}

// ========================================================================
// PUBLISH HELPERS
// ========================================================================

std::string PropArmHardware::nsTopic(const std::string &ns, const std::string &topic) const
{
    if (ns.empty()) return topic;
    if (ns.back() == '/') return ns + topic;
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

void PropArmHardware::publishMotorTelemetry(double motor_speed, std::uint16_t pwm_snapshot)
{
    if (motor_speed_pub_)
    {
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = motor_speed;
        motor_speed_pub_->publish(speed_msg);
    }

    if (vpwm_pub_)
    {
        std_msgs::msg::UInt16 pwm_msg;
        pwm_msg.data = pwm_snapshot;
        vpwm_pub_->publish(pwm_msg);
    }
}

void PropArmHardware::publishJointTelemetry()
{
    auto it = joints_.find(joint_name_);
    if (it != joints_.end() && arm_angle_pub_)
    {
        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = it->second.position_output;
        arm_angle_pub_->publish(angle_msg);
    }
}

// ========================================================================
// RUNTIME PARAMS
// ========================================================================

void PropArmHardware::declareRuntimeParams_()
{
    // Motor model (live)
    nh_->declare_parameter<double>("motor_model.Kw", Kw_);
    nh_->declare_parameter<double>("motor_model.tau_w", tau_w_);

    nh_->declare_parameter<double>("motor_model.c1", c1_);
    nh_->declare_parameter<double>("motor_model.c2", c2_);
    nh_->declare_parameter<double>("motor_model.d1", d1_);
    nh_->declare_parameter<double>("motor_model.d2", d2_);

    nh_->declare_parameter<double>("motor_model.L_w", L_w_);
    nh_->declare_parameter<double>("motor_model.Ts", Ts_);
    nh_->declare_parameter<int>("motor_model.pwm_ref_us", static_cast<int>(pwm_ref_us_));
    nh_->declare_parameter<int>("motor_model.pwm_min_us", static_cast<int>(pwm_min_us_));
    nh_->declare_parameter<int>("motor_model.pwm_max_us", static_cast<int>(pwm_max_us_));
    nh_->declare_parameter<int>("motor_model.pwm_spin_min_us", static_cast<int>(pwm_spin_min_us_));
    nh_->declare_parameter<double>("motor_model.motor_cmd_scale", motor_cmd_scale_);

    // Runtime “physics-like” params (live)
    nh_->declare_parameter<double>("runtime.viscous_arm", rt_viscous_arm_.load());
    nh_->declare_parameter<double>("runtime.coulomb_arm", rt_coulomb_arm_.load());
    nh_->declare_parameter<double>("runtime.thrust_k", rt_thrust_k_.load());
}

rcl_interfaces::msg::SetParametersResult
PropArmHardware::onParams_(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    bool motor_changed = false;

    // legacy
    double new_Kw = Kw_;
    double new_tau = tau_w_;

    // preferred
    double new_c1 = c1_;
    double new_c2 = c2_;
    double new_d1 = d1_;
    double new_d2 = d2_;

    double new_Lw = L_w_;
    double new_Ts = Ts_;
    int new_pwm_ref = static_cast<int>(pwm_ref_us_);
    int new_pwm_min = static_cast<int>(pwm_min_us_);
    int new_pwm_max = static_cast<int>(pwm_max_us_);
    int new_pwm_spin = static_cast<int>(pwm_spin_min_us_);
    double new_scale = motor_cmd_scale_;

    for (const auto &p : params)
    {
        const auto &name = p.get_name();

        // Motor model
        if (name == "motor_model.Kw")
        {
            new_Kw = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.tau_w")
        {
            new_tau = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.c1")
        {
            new_c1 = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.c2")
        {
            new_c2 = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.d1")
        {
            new_d1 = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.d2")
        {
            new_d2 = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.L_w")
        {
            new_Lw = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.Ts")
        {
            new_Ts = p.as_double();
            motor_changed = true;
        }
        else if (name == "motor_model.pwm_ref_us")
        {
            new_pwm_ref = p.as_int();
            motor_changed = true;
        }
        else if (name == "motor_model.pwm_min_us")
        {
            new_pwm_min = p.as_int();
            motor_changed = true;
        }
        else if (name == "motor_model.pwm_max_us")
        {
            new_pwm_max = p.as_int();
            motor_changed = true;
        }
        else if (name == "motor_model.pwm_spin_min_us")
        {
            new_pwm_spin = p.as_int();
            motor_changed = true;
        }
        else if (name == "motor_model.motor_cmd_scale")
        {
            new_scale = p.as_double();
            motor_changed = true;
        }

        // Runtime “physics-like”
        else if (name == "runtime.viscous_arm")
        {
            rt_viscous_arm_.store(p.as_double());
        }
        else if (name == "runtime.coulomb_arm")
        {
            rt_coulomb_arm_.store(p.as_double());
        }
        else if (name == "runtime.thrust_k")
        {
            rt_thrust_k_.store(p.as_double());
        }
    }

    // Validate
    if (new_Ts <= 0.0)
    {
        res.successful = false;
        res.reason = "Ts must be > 0";
        return res;
    }
    if (new_pwm_min < 0 || new_pwm_max < 0 || new_pwm_min >= new_pwm_max)
    {
        res.successful = false;
        res.reason = "Invalid pwm_min_us/pwm_max_us";
        return res;
    }
    if (new_pwm_spin < new_pwm_min || new_pwm_spin > new_pwm_max)
    {
        res.successful = false;
        res.reason = "pwm_spin_min_us must be within [pwm_min_us, pwm_max_us]";
        return res;
    }

    // Apply
    {
        std::scoped_lock lk(motor_mtx_);

        Kw_ = new_Kw;
        tau_w_ = new_tau;

        c1_ = new_c1;
        c2_ = new_c2;
        d1_ = new_d1;
        d2_ = new_d2;

        L_w_ = new_Lw;
        Ts_ = new_Ts;

        pwm_min_us_ = static_cast<std::uint16_t>(new_pwm_min);
        pwm_max_us_ = static_cast<std::uint16_t>(new_pwm_max);
        pwm_ref_us_ = static_cast<std::uint16_t>(new_pwm_ref);
        pwm_spin_min_us_ = static_cast<std::uint16_t>(new_pwm_spin);
        motor_cmd_scale_ = new_scale;

        if (motor_changed)
        {
            // Rebuild motor model while preserving current speed state
            const double w0 = motor_model_.getSpeedRadSec();
            motor_model_ = prop_arm_characterization::MotorSpeedModel(
                               Ts_,
                               L_w_,
                               c1_, c2_, d1_, d2_,
                               pwm_min_us_,
                               pwm_max_us_,
                               pwm_spin_min_us_,
                               motor_cmd_scale_,
                               w0,
                               prop_radius_m_);
        }

        // Keep current PWM consistent
        const int clamped = std::clamp(
                                static_cast<int>(current_pwm_us_),
                                static_cast<int>(pwm_min_us_),
                                static_cast<int>(pwm_max_us_));
        current_pwm_us_ = static_cast<std::uint16_t>(clamped);
    }

    return res;
}

double PropArmHardware::signNoZero_(double x) noexcept
{
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

} // namespace prop_arm_gazebo_control

PLUGINLIB_EXPORT_CLASS(prop_arm_gazebo_control::PropArmHardware,
                       gz_ros2_control::GazeboSimSystemInterface)

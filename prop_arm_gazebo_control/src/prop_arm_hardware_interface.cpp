#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace prop_arm_gazebo_control
{

    bool PropArmHardware::initSim(rclcpp::Node::SharedPtr &model_nh,
                                  std::map<std::string, sim::Entity> &joints,
                                  const hardware_interface::HardwareInfo &hardware_info,
                                  sim::EntityComponentManager &ecm,
                                  unsigned int /*update_rate*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                    "Initializing PropArm Hardware (Vref -> PWM square + averaged EMF model).");

        nh_ = model_nh;
        ecm_ = &ecm;
        enabled_joints_ = joints;

        // --- parámetros desde URDF ---
        if (auto it = hardware_info.hardware_parameters.find("robot_namespace");
            it != hardware_info.hardware_parameters.end())
            robot_namespace_ = it->second;

        if (auto it = hardware_info.hardware_parameters.find("actuator_index");
            it != hardware_info.hardware_parameters.end())
            actuator_index_ = std::max(0, std::stoi(it->second));

        if (auto it = hardware_info.hardware_parameters.find("max_rot_velocity_radps");
            it != hardware_info.hardware_parameters.end())
            max_rot_vel_ = std::max(0.0, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("V_bus");
            it != hardware_info.hardware_parameters.end())
            V_bus = std::max(0.0, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("K_e");
            it != hardware_info.hardware_parameters.end())
            K_e = std::max(1e-12, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("K_m");
            it != hardware_info.hardware_parameters.end())
            K_m = std::max(1e-12, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("R_m");
            it != hardware_info.hardware_parameters.end())
            R_m = std::max(1e-12, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("R_s");
            it != hardware_info.hardware_parameters.end())
            R_s = std::max(1e-12, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("K_f");
            it != hardware_info.hardware_parameters.end())
            K_f = std::max(0.0, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("J_m");
            it != hardware_info.hardware_parameters.end())
            J_m = std::max(1e-12, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("vref_min");
            it != hardware_info.hardware_parameters.end())
            vref_min_ = std::max(0.0, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("vref_max");
            it != hardware_info.hardware_parameters.end())
            vref_max_ = std::max(0.0, std::stod(it->second));

        if (auto it = hardware_info.hardware_parameters.find("duty_max");
            it != hardware_info.hardware_parameters.end())
            duty_max_ = std::clamp(std::stod(it->second), 0.0, 1.0);

        if (auto it = hardware_info.hardware_parameters.find("pwm_frequency_hz");
            it != hardware_info.hardware_parameters.end())
            pwm_frequency_hz_ = std::max(1.0, std::stod(it->second));

        // --- tópicos con namespace ---
        actuators_topic_ = nsTopic(robot_namespace_, "command/motor_speed");
        auto cmd_vref_topic = nsTopic(robot_namespace_, "cmd/vref"); // publicas tú (V)
        auto vemf_topic = nsTopic(robot_namespace_, "vemf");         // V_emf (V)
        auto duty_topic = nsTopic(robot_namespace_, "duty");         // duty [-]
        auto vpwm_topic = nsTopic(robot_namespace_, "vpwm");         // PWM cuadrada 0/V_bus

        // GZ transport
        gz_node_ = std::make_unique<gz::transport::Node>();
        actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);
        RCLCPP_INFO(nh_->get_logger(), "Actuators topic: %s (act_idx=%d, vmax=%.2f rad/s)",
                    actuators_topic_.c_str(), actuator_index_, max_rot_vel_);

        // Publicadores
        motor_speed_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            "/prop_arm/motor_speed_est", rclcpp::QoS(100).best_effort());
        arm_angle_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            "/prop_arm/arm_angle_deg", rclcpp::QoS(50).best_effort());
        vemf_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            vemf_topic, rclcpp::QoS(100).best_effort());
        duty_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            duty_topic, rclcpp::QoS(50).best_effort());
        vpwm_pub_ = nh_->create_publisher<std_msgs::msg::Float64>(
            vpwm_topic, rclcpp::QoS(400).best_effort()); // alta tasa para ver pulsos si bajas f_pwm

        // Suscripción de Vref
        vref_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
            cmd_vref_topic, rclcpp::QoS(10).reliable(),
            std::bind(&PropArmHardware::onVrefMsg, this, std::placeholders::_1));

        // Interfaces ros2_control
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

            if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointPosition().TypeId()))
                ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointPosition());
            if (!ecm.EntityHasComponentType(jd.sim_joint, gz::sim::components::JointVelocity().TypeId()))
                ecm.CreateComponent(jd.sim_joint, gz::sim::components::JointVelocity());

            state_interfaces_.emplace_back(name, hardware_interface::HW_IF_POSITION, &jd.position);
            state_interfaces_.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &jd.velocity);
            command_interfaces_.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &jd.velocity_command);

            RCLCPP_INFO(nh_->get_logger(), "Joint ready: %s", name.c_str());
        }

        RCLCPP_INFO(nh_->get_logger(),
                    "Hardware ready - %zu joints | V_bus=%.2f V | f_pwm=%.1f Hz (averaged EMF).",
                    joints_.size(), V_bus, pwm_frequency_hz_);
        return true;
    }

    hardware_interface::CallbackReturn
    PropArmHardware::on_init(const hardware_interface::HardwareInfo & /*info*/)
    {
        // No invocamos a SystemInterface::on_init(info) (evita warning deprecado).
        return hardware_interface::CallbackReturn::SUCCESS;
    }

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

    hardware_interface::CallbackReturn
    PropArmHardware::on_activate(const rclcpp_lifecycle::State &)
    {
        for (auto &kv : joints_)
        {
            kv.second.position = 0.0;
            kv.second.velocity = 0.0;
            kv.second.velocity_command = 0.0;
        }

        // Detener motor
        gz::msgs::Actuators msg;
        msg.mutable_velocity()->Resize(actuator_index_ + 1, 0.0);
        actuators_pub_.Publish(msg);

        // Reset internos
        pwm_phase_ = 0.0;
        duty_eff_ = 0.0;
        vpwm_sq_ = 0.0;
        vemf_ = 0.0;
        omega_est_ = 0.0;

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
        const double dt = std::max(0.0, period.seconds());

        // 1) Duty a partir de Vref (promedio)
        double vref_local = 0.0;
        {
            std::scoped_lock lk(io_mtx_);
            vref_local = std::clamp(vref_cmd_, vref_min_, vref_max_);
        }
        duty_eff_ = (V_bus > 1e-12) ? std::clamp(vref_local / V_bus, 0.0, duty_max_) : 0.0;

        // 2) PWM cuadrada para visualización (0 / V_bus)
        if (dt > 0.0)
            stepPwm(dt);
        vpwm_sq_ = (pwm_phase_ < duty_eff_) ? V_bus : 0.0;

        // 3) Modelo eléctrico promediado (dinámica 1er orden sobre v_emf)
        const double v_pwm_avg = duty_eff_ * V_bus;
        if (dt > 0.0)
            integrateEmf(dt, v_pwm_avg);

        // 4) Velocidad estimada (constante en estacionario si Vref es constante)
        omega_est_ = (K_e > 0.0) ? (vemf_ / K_e) : 0.0;
        const double omega_cmd = std::clamp(omega_est_, 0.0, max_rot_vel_);

        // 5) Telemetría y mando a Gazebo
        publishTelemetry();

        gz::msgs::Actuators msg;
        msg.mutable_velocity()->Resize(actuator_index_ + 1, 0.0);
        msg.set_velocity(actuator_index_, omega_cmd);
        if (!actuators_pub_.Publish(msg))
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PropArmHardware"), *nh_->get_clock(), 1000,
                                 "Failed to publish motor command");
        }

        return hardware_interface::return_type::OK;
    }

    void PropArmHardware::onVrefMsg(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (!msg)
            return;
        std::scoped_lock lk(io_mtx_);
        vref_cmd_ = msg->data;
    }

    void PropArmHardware::stepPwm(double dt)
    {
        const double T = 1.0 / pwm_frequency_hz_;
        pwm_phase_ += dt / T;
        pwm_phase_ -= std::floor(pwm_phase_); // wrap a [0,1)
    }

    void PropArmHardware::integrateEmf(double dt, double v_pwm_avg)
    {
        // dv_emf/dt = ( alpha*(v_pwm_avg - v_emf) - K_f*v_emf ) / J_m
        const double alpha = (K_m * K_e) / (R_s + R_m); // [N*m*s/V] -> consistente con unidades
        const double a = (alpha / J_m);
        const double b = (K_f / J_m);

        // Integrador explícito estable para dt pequeño (frecuencia de update ros2_control)
        const double dv = (a * (v_pwm_avg - vemf_) - b * vemf_) * dt;
        vemf_ += dv;

        // Seguridad numérica
        if (vemf_ < 0.0)
            vemf_ = 0.0;
        if (vemf_ > V_bus)
            vemf_ = V_bus;
    }

    void PropArmHardware::publishTelemetry()
    {
        std_msgs::msg::Float64 m;

        // Señales de planta
        m.data = vemf_;
        vemf_pub_->publish(m);
        m.data = duty_eff_;
        duty_pub_->publish(m);
        m.data = vpwm_sq_;
        vpwm_pub_->publish(m);

        // Velocidad estimada y ángulo (como lo tenías)
        m.data = omega_est_;
        motor_speed_pub_->publish(m);

        auto it = joints_.find("arm_link_joint");
        if (it != joints_.end())
        {
            std_msgs::msg::Float64 ang;
            ang.data = it->second.position * 180.0 / M_PI;
            arm_angle_pub_->publish(ang);
        }
    }

} // namespace prop_arm_gazebo_control

PLUGINLIB_EXPORT_CLASS(prop_arm_gazebo_control::PropArmHardware,
                       gz_ros2_control::GazeboSimSystemInterface)

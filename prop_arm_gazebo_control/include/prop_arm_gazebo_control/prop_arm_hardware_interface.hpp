#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <mutex>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

namespace sim = gz::sim;

namespace prop_arm_gazebo_control
{
    struct JointData
    {
        double position{0.0};                    // [rad]
        double velocity{0.0};                    // [rad/s]
        double velocity_command{0.0};            // [rad/s] (compatibilidad ros2_control)
        sim::Entity sim_joint{sim::kNullEntity}; // joint en simulación
    };

    class PropArmHardware : public gz_ros2_control::GazeboSimSystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

        bool initSim(rclcpp::Node::SharedPtr &model_nh,
                     std::map<std::string, sim::Entity> &joints,
                     const hardware_interface::HardwareInfo &hardware_info,
                     sim::EntityComponentManager &ecm,
                     unsigned int update_rate) override;

        // Evita warning deprecado (no llamamos al on_init(base))
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &period) override;

    private:
        static std::string nsTopic(const std::string &ns, const std::string &leaf)
        {
            if (ns.empty())
                return "/" + leaf;
            if (ns.front() == '/')
                return ns + "/" + leaf;
            return "/" + ns + "/" + leaf;
        }

        // --- Config ---
        std::string robot_namespace_{"prop_arm"};
        std::string actuators_topic_{"/prop_arm/command/motor_speed"};
        int actuator_index_{0};
        double max_rot_vel_{785.0}; // [rad/s]

        // Parámetros eléctricos (modelo promediado)
        double V_bus{12.0}; // [V]
        double K_e{5.5e-3}; // [V/(rad/s)]
        double K_m{5.5e-3}; // [N*m/A]
        double R_m{1.0};    // [Ohm]
        double R_s{1.0};    // [Ohm]
        double K_f{1.0e-5}; // [N*m*s/V]  (amortiguamiento equivalente)
        double J_m{3.0e-6}; // [kg*m^2]

        // Límites para Vref y duty
        double vref_min_{0.0};  // [V]
        double vref_max_{12.0}; // [V]
        double duty_max_{0.95}; // [-]

        // PWM cuadrado (solo para visualización)
        double pwm_frequency_hz_{500.0}; // [Hz]
        double pwm_phase_{0.0};          // [0,1)

        // Estado interno
        std::mutex io_mtx_;
        double vref_cmd_{0.0};  // [V] referencia promedio recibida
        double duty_eff_{0.0};  // [-]
        double vpwm_sq_{0.0};   // [V] cuadrada publicada (0 o V_bus)
        double vemf_{0.0};      // [V] estado eléctrico (promediado)
        double omega_est_{0.0}; // [rad/s]

        // ROS / Gazebo
        rclcpp::Node::SharedPtr nh_;

        // Telemetría existente
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_speed_pub_; // /prop_arm/motor_speed_est
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_angle_pub_;   // /prop_arm/arm_angle_deg

        // Telemetría de planta
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vemf_pub_; // /<ns>/vemf
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duty_pub_; // /<ns>/duty
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vpwm_pub_; // /<ns>/vpwm

        // Entrada de planta
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vref_sub_; // /<ns>/cmd/vref

        sim::EntityComponentManager *ecm_{nullptr};
        std::map<std::string, sim::Entity> enabled_joints_;
        std::unique_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher actuators_pub_;

        // ros2_control
        std::unordered_map<std::string, JointData> joints_;
        std::vector<hardware_interface::StateInterface> state_interfaces_;
        std::vector<hardware_interface::CommandInterface> command_interfaces_;

        // Helpers
        void onVrefMsg(const std_msgs::msg::Float64::SharedPtr msg);
        void stepPwm(double dt);
        void integrateEmf(double dt, double v_pwm_avg);
        void publishTelemetry();
    };

} // namespace prop_arm_gazebo_control

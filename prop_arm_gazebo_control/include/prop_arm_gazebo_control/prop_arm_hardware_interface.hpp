#pragma once
#include <memory> // IWYU pragma: keep
#include <string>
#include <unordered_map> // IWYU pragma: keep
#include <vector>        // IWYU pragma: keep
#include <algorithm>     // IWYU pragma: keep
#include <map>
#include <cstdint>
#include <deque>
#include <atomic>
#include <mutex>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "prop_arm_characterization/motor_speed_model.hpp"
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

namespace sim = gz::sim;

namespace prop_arm_gazebo_control
{

struct JointData
{
    sim::Entity sim_joint{sim::kNullEntity};
    double position_output{0.0};
    double velocity_output{0.0};
    double command_effort{0.0};
};

class PropArmHardware : public gz_ros2_control::GazeboSimSystemInterface
{
public:
    PropArmHardware() = default;
    ~PropArmHardware() override = default;

    bool initSim(rclcpp::Node::SharedPtr &model_nh,
                 std::map<std::string, sim::Entity> &joints,
                 const hardware_interface::HardwareInfo &hardware_info,
                 sim::EntityComponentManager &ecm,
                 unsigned int update_rate) override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time,
                                         const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) override;

private:
    std::string nsTopic(const std::string &ns, const std::string &topic) const;

    void loadParameters(const hardware_interface::HardwareInfo &hardware_info);
    void setupControlInterfaces(const hardware_interface::HardwareInfo &hardware_info,
                                sim::EntityComponentManager &ecm);

    void publishToGazebo(double motor_speed);
    void publishMotorTelemetry(double motor_speed, std::uint16_t pwm_snapshot);
    void publishJointTelemetry();

    // CONFIGURATION PARAMETERS
    std::string robot_namespace_{"prop_arm"};
    std::string actuators_topic_{"/prop_arm/command/motor_speed"};
    std::string joint_name_{"arm_link_joint"};

    // Motor model parameters
    prop_arm_characterization::MotorSpeedModel motor_model_;
    unsigned int actuator_index_{0};
    double prop_radius_m_{0.1};           // Propeller radius [m]

    // Legacy (kept for compatibility; ARX parameters are preferred now)
    double Kw_{0.0};                      // Motor gain [rad/s/V]
    double tau_w_{0.0};                   // Motor time constant [s]

    // NEW: ARX(2,2) coefficients for discrete motor speed model (Ts-fixed)
    double c1_{1.9823};
    double c2_{-0.9825};
    double d1_{1.12e-4};
    double d2_{1.11e-4};

    double L_w_{0.10};                    // Motor delay [s]
    double Ts_{0.01};                     // Sampling time [s]
    std::uint16_t pwm_spin_min_us_{0};
    std::uint16_t pwm_ref_us_{1500};      // Reference PWM [us]
    std::uint16_t pwm_max_us_{2000};      // Max PWM limit [us]
    std::uint16_t pwm_min_us_{1000};      // Min PWM limit [us]
    std::uint16_t current_pwm_us_{1500};  // Current PWM command [us]
    double motor_cmd_scale_{1.0};

    // ROS2 & GAZEBO COMMUNICATION
    rclcpp::Node::SharedPtr nh_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_speed_pub_; // Motor speed [rad/s]
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_angle_pub_;   // Arm angle [deg]
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr vpwm_pub_;         // PWM feedback [us]

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr pwm_cmd_sub_;   // PWM command [us]

    // Gazebo transport
    sim::EntityComponentManager *ecm_{nullptr};
    std::map<std::string, sim::Entity> enabled_joints_;
    std::unique_ptr<gz::transport::Node> gz_node_;
    gz::transport::Node::Publisher actuators_pub_;

    // ROS2_CONTROL DATA
    std::unordered_map<std::string, JointData> joints_;
    std::vector<hardware_interface::StateInterface> state_interfaces_;
    std::vector<hardware_interface::CommandInterface> command_interfaces_;

    // Runtime tunables (live)
    std::atomic<double> rt_viscous_arm_{0.0};
    std::atomic<double> rt_coulomb_arm_{0.0};
    std::atomic<double> rt_thrust_k_{1.0};
    std::mutex motor_mtx_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    void declareRuntimeParams_();
    rcl_interfaces::msg::SetParametersResult onParams_(const std::vector<rclcpp::Parameter> &params);
    static double signNoZero_(double x) noexcept;
};

} // namespace prop_arm_gazebo_control

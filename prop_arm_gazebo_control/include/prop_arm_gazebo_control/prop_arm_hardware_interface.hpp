#pragma once
#include <memory> // IWYU pragma: keep
#include <string>
#include <unordered_map> // IWYU pragma: keep
#include <vector>        // IWYU pragma: keep
#include <algorithm>     // IWYU pragma: keep
#include <map>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <prop_arm_gazebo_control/motor_speed_model.hpp>
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

namespace sim = gz::sim;

namespace prop_arm_gazebo_control
{
    /// @brief Joint data structure for storing joint state and command
    struct JointData
    {
        double position_output{0.0};             // [rad]
        double velocity_output{0.0};             // [rad/s]
        sim::Entity sim_joint{sim::kNullEntity}; // Gazebo simulation joint entity
    };

    /// @brief Hardware interface for PropArm robot in Gazebo simulation
    /// Handles motor control via PWM commands and publishes joint telemetry
    class PropArmHardware : public gz_ros2_control::GazeboSimSystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PropArmHardware)

        // LIFECYCLE INTERFACE
        bool initSim(rclcpp::Node::SharedPtr &model_nh,
                     std::map<std::string, sim::Entity> &joints,
                     const hardware_interface::HardwareInfo &hardware_info,
                     sim::EntityComponentManager &ecm,
                     unsigned int update_rate) override;
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // ROS2_CONTROL INTERFACE
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    protected:
        // ====================================================================
        // UTILITY FUNCTIONS
        // ====================================================================

        /// @brief Creates namespaced topic name
        static std::string nsTopic(const std::string &ns, const std::string &leaf)
        {
            if (ns.empty())
                return "/" + leaf;
            if (ns.front() == '/')
                return ns + "/" + leaf;
            return "/" + ns + "/" + leaf;
        }

        // SETUP HELPERS
        void loadParameters(const hardware_interface::HardwareInfo &hardware_info);
        void setupGazeboTransport();
        void setupROSPublishers();
        void setupROSSubscribers();
        void setupControlInterfaces(const hardware_interface::HardwareInfo &hardware_info,
                                    sim::EntityComponentManager &ecm);

        // PUBLISHING HELPERS
        void publishToGazebo(double motor_speed);
        void publishMotorTelemetry(double motor_speed);
        void publishJointTelemetry();

        // CONFIGURATION PARAMETERS
        std::string robot_namespace_{"prop_arm"};
        std::string actuators_topic_{"/prop_arm/command/motor_speed"};
        std::string joint_name_{"arm_link_joint"};

        // Motor model parameters
        MotorSpeedModel motor_model_;
        unsigned int actuator_index_{0};
        double prop_radius_m_{0.1}; // [m]
        double Kw_{0.0};            // Motor gain [rad/s/V]
        double tau_w_{0.0};         // Motor time constant [s]
        double Ts_{0.01};           // Sampling time [s]
        int pwm_ref_us_{1500};      // Reference PWM [us]
        int pwm_max_us_{2000};      // Max PWM limit [us]
        int pwm_min_us_{1000};      // Min PWM limit [us]
        int current_pwm_us_{1500};  // Current PWM command [us]

        // ROS2 & GAZEBO COMMUNICATION
        rclcpp::Node::SharedPtr nh_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_speed_pub_; // Motor speed [rad/s]
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_angle_pub_;   // Arm angle [deg]
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vpwm_pub_;        // PWM feedback [us]

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pwm_cmd_sub_; // PWM command [us]

        // Gazebo transport
        sim::EntityComponentManager *ecm_{nullptr};
        std::map<std::string, sim::Entity> enabled_joints_;
        std::unique_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher actuators_pub_;

        // ROS2_CONTROL DATA
        std::unordered_map<std::string, JointData> joints_;
        std::vector<hardware_interface::StateInterface> state_interfaces_;
        std::vector<hardware_interface::CommandInterface> command_interfaces_;
    };

} // namespace prop_arm_gazebo_control
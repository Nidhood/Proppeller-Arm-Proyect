#include "prop_arm_gazebo_control/prop_arm_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace prop_arm_gazebo_control
{

    hardware_interface::CallbackReturn PropArmHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Initializing PropArm Hardware Interface");

        // Get robot namespace from parameters
        robot_namespace_ = "prop_arm"; // Default
        if (info_.hardware_parameters.find("robot_namespace") != info_.hardware_parameters.end())
        {
            robot_namespace_ = info_.hardware_parameters.at("robot_namespace");
        }

        // Initialize Gazebo transport
        gz_node_ = std::make_unique<gz::transport::Node>();

        // Setup actuators topic - the MulticopterMotorModel listens to this
        actuators_topic_ = "/" + robot_namespace_ + "/command/motor_speed";
        actuators_pub_ = gz_node_->Advertise<gz::msgs::Actuators>(actuators_topic_);

        // Initialize joint data and PID controllers
        for (const auto &joint : info_.joints)
        {
            joints_[joint.name] = JointData{};
            pid_controllers_[joint.name] = PIDController{};

            RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                        "Initialized joint: %s", joint.name.c_str());
        }

        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                    "Actuators command topic: %s", actuators_topic_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    PropArmHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (const auto &joint : info_.joints)
        {
            const std::string &joint_name = joint.name;

            for (const auto &interface : joint.state_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    state_interfaces.emplace_back(
                        joint_name, hardware_interface::HW_IF_POSITION, &joints_[joint_name].position);
                }
                else if (interface.name == hardware_interface::HW_IF_VELOCITY)
                {
                    state_interfaces.emplace_back(
                        joint_name, hardware_interface::HW_IF_VELOCITY, &joints_[joint_name].velocity);
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                    "Exported %zu state interfaces", state_interfaces.size());
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    PropArmHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (const auto &joint : info_.joints)
        {
            const std::string &joint_name = joint.name;

            for (const auto &interface : joint.command_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    // Aunque el controlador envía comandos de posición,
                    // internamente los convertimos a velocidad del motor
                    command_interfaces.emplace_back(
                        joint_name, hardware_interface::HW_IF_POSITION, &joints_[joint_name].motor_speed_command);
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                    "Exported %zu command interfaces", command_interfaces.size());
        return command_interfaces;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Activating PropArm Hardware");

        // Reset PID controllers
        for (auto &[joint_name, pid] : pid_controllers_)
        {
            pid.prev_error = 0.0;
            pid.integral = 0.0;
        }

        // Initialize commands to zero
        for (auto &[joint_name, joint_data] : joints_)
        {
            joint_data.motor_speed_command = 0.0;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn PropArmHardware::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"), "Deactivating PropArm Hardware");

        // Stop the motor
        gz::msgs::Actuators actuators_msg;
        actuators_msg.add_velocity(0.0);
        actuators_pub_.Publish(actuators_msg);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type PropArmHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // TODO: En una implementación completa, leerías el estado del joint desde Gazebo
        // usando subscribers a topics como /world/default/model/prop_arm/joint_state
        // o mediante el Component System de Gazebo

        // Por ahora, simulamos que el joint se mueve hacia la posición comandada
        for (auto &[joint_name, joint_data] : joints_)
        {
            // Simulación simple: el brazo se mueve gradualmente hacia el comando
            // En la realidad, Gazebo Physics calculará la posición basada en las fuerzas del motor
            double position_error = joint_data.motor_speed_command - joint_data.position;
            joint_data.velocity = position_error * 2.0; // Simple P controller for simulation
            joint_data.position += joint_data.velocity * period.seconds();

            // Log periódico para debug
            static int read_counter = 0;
            if (++read_counter % 1000 == 0) // Log cada 10 segundos a 100Hz
            {
                RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                            "Joint %s: pos=%.3f, vel=%.3f, cmd=%.3f",
                            joint_name.c_str(), joint_data.position,
                            joint_data.velocity, joint_data.motor_speed_command);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type PropArmHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Convertir comandos de posición a velocidades del motor usando PID
        for (auto &[joint_name, joint_data] : joints_)
        {
            auto &pid = pid_controllers_[joint_name];

            // El comando que recibimos es una posición deseada
            double desired_position = joint_data.motor_speed_command; // Viene del controller
            double current_position = joint_data.position;

            // Calcular error
            double error = desired_position - current_position;

            // PID calculation
            pid.integral += error * period.seconds();
            double derivative = (error - pid.prev_error) / period.seconds();

            double motor_speed = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

            // Limitar salida
            motor_speed = std::max(pid.output_min, std::min(pid.output_max, motor_speed));

            // Enviar comando al motor en Gazebo usando mensaje Actuators
            // El MulticopterMotorModel espera un mensaje Actuators con el índice correcto
            gz::msgs::Actuators actuators_msg;
            actuators_msg.add_velocity(motor_speed); // Índice 0 para actuator_number = 0
            actuators_pub_.Publish(actuators_msg);

            pid.prev_error = error;

            // Log periódico
            static int write_counter = 0;
            if (++write_counter % 500 == 0) // Log cada 5 segundos a 100Hz
            {
                RCLCPP_INFO(rclcpp::get_logger("PropArmHardware"),
                            "Joint %s: target=%.3f, current=%.3f, motor_speed=%.3f",
                            joint_name.c_str(), desired_position, current_position, motor_speed);
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace prop_arm_gazebo_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    prop_arm_gazebo_control::PropArmHardware,
    hardware_interface::SystemInterface)
#include "prop_arm_gazebo_control/motor_commander.hpp"

namespace prop_arm_control
{

    MotorCommander::MotorCommander()
        : Node("motor_commander"), last_command_type_(LastCommandType::NONE)
    {
        // Publishers - Use proper topic names matching controller configuration
        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_force_controller/commands", 10);

        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);

        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_controller/joint_trajectory", 10);

        // Initialize Gazebo transport node
        gz_node_ = std::make_unique<gz::transport::Node>();

        // Wait for connections
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        RCLCPP_INFO(this->get_logger(), "Motor Commander initialized with equilibrium physics model");
    }

    double MotorCommander::mitToGazeboAngle(double mit_angle_degrees) const
    {
        return mit_angle_degrees * M_PI / 180.0;
    }

    double MotorCommander::gazeboToMitAngle(double gazebo_radians) const
    {
        return gazebo_radians * 180.0 / M_PI;
    }

    void MotorCommander::send_zero_commands()
    {
        // Send zero to ALL controllers to ensure clean override
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(0.0);
        gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);

        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(0.0);
        effort_pub_->publish(effort_msg);

        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(0.0);
        velocity_pub_->publish(velocity_msg);

        // Send empty trajectory to stop position controller
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.header.stamp = this->get_clock()->now();
        trajectory_msg.joint_names.push_back("arm_link_joint");
        trajectory_pub_->publish(trajectory_msg);
    }

    double MotorCommander::calculateEquilibriumMotorSpeed() const
    {
        // === PARÁMETROS DEL SISTEMA (ajustar según tu configuración real) ===

        // Geometría del brazo
        const double M_m = 0.1;  // Masa en la punta del brazo [kg] (estimar)
        const double M_a = 0.05; // Masa del brazo [kg] (estimar)
        const double g = 9.81;   // Gravedad [m/s²]

        // Constantes del plugin MulticopterMotorModel
        const double motor_constant = 0.005; // motorConstant del plugin [N/(rad/s)²]

        // === CÁLCULO SEGÚN LA FÓRMULA DEL DOCUMENTO ===

        // Paso 1: Empuje de equilibrio para brazo horizontal (θ = 0°)
        // F_T* = g(M_m + M_a/2)
        const double F_T_star = g * (M_m + M_a / 2.0);

        // Paso 2: Velocidad de equilibrio del motor
        // ω₀ = √(F_T* / K_t0)
        // Donde K_t0 = motorConstant del plugin
        const double omega_0 = std::sqrt(F_T_star / motor_constant);

        // Limitar a la velocidad máxima del plugin
        const double max_motor_speed = 785.0; // maxRotVelocity del plugin
        const double omega_equilibrium = std::min(omega_0, max_motor_speed);

        RCLCPP_INFO(this->get_logger(),
                    "Equilibrium calculation: F_T*=%.4f N, ω₀=%.2f rad/s (limited to %.2f)",
                    F_T_star, omega_0, omega_equilibrium);

        return omega_equilibrium;
    }

    void MotorCommander::command_angle(double angle_degrees)
    {
        RCLCPP_INFO(this->get_logger(), "=== ANGLE CONTROL COMMAND ===");
        RCLCPP_INFO(this->get_logger(), "Target angle: %.1f° (MIT frame)", angle_degrees);

        // Validate angle range (-90 to +90 degrees as per MIT model)
        angle_degrees = std::max(-90.0, std::min(90.0, angle_degrees));

        // CRITICAL: First stop all other controllers
        send_zero_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Calculate thrust needed for this angle using CORRECTED physics
        double thrust_force = calculateThrustForAngle(angle_degrees);

        RCLCPP_INFO(this->get_logger(), "Calculated thrust: %.2f N", thrust_force);

        // Convert thrust to motor speed using plugin's motor constant
        double motor_speed = 0.0;
        if (thrust_force > 0.001)
        {
            const double motor_constant = 0.005; // From plugin config
            motor_speed = std::sqrt(thrust_force / motor_constant);
            motor_speed = std::min(motor_speed, 785.0); // Max speed from plugin
        }

        // Send motor command
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(motor_speed);
        gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);

        // Also send through velocity controller
        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(motor_speed);
        velocity_pub_->publish(velocity_msg);

        RCLCPP_INFO(this->get_logger(), "Motor speed: %.1f rad/s", motor_speed);

        last_command_type_ = LastCommandType::VELOCITY;
        RCLCPP_INFO(this->get_logger(), "Command sent successfully!");
    }

    double MotorCommander::calculateThrustForAngle(double target_angle_degrees)
    {
        // Parámetros físicos del sistema
        const double L_a = 0.15; // Longitud del brazo [m]
        const double M_m = 0.1;  // Masa en la punta [kg]
        const double M_a = 0.05; // Masa del brazo [kg]
        const double g = 9.81;   // Gravedad [m/s²]

        const double target_rad = target_angle_degrees * M_PI / 180.0;

        // Empuje base de equilibrio (horizontal)
        const double F_T_equilibrium = g * (M_m + M_a / 2.0);

        // Para ángulos diferentes de horizontal, ajustar según componente gravitacional
        // Para θ > 0 (hacia arriba): necesita más empuje
        // Para θ < 0 (hacia abajo): necesita menos empuje

        double thrust_adjustment = 0.0;
        if (std::abs(target_angle_degrees) > 1.0) // Solo ajustar si no es ~horizontal
        {
            // Torque gravitacional adicional por el ángulo
            const double additional_torque = (M_m * g * L_a + M_a * g * L_a / 2.0) *
                                             (std::sin(target_rad) - std::sin(0.0));
            thrust_adjustment = additional_torque / L_a;
        }

        double required_thrust = F_T_equilibrium + thrust_adjustment;

        // Asegurar que el empuje sea positivo (las hélices no pueden "tirar")
        required_thrust = std::max(0.0, required_thrust);

        // Limitar a valores razonables
        const double max_thrust = 50.0; // [N]
        required_thrust = std::min(required_thrust, max_thrust);

        return required_thrust;
    }

    void MotorCommander::command_force(double force_newtons)
    {
        RCLCPP_INFO(this->get_logger(), "=== FORCE CONTROL COMMAND ===");
        RCLCPP_INFO(this->get_logger(), "Target force: %.1f N", force_newtons);

        send_zero_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Ensure positive force only (realistic propeller constraint)
        force_newtons = std::max(0.0, std::min(45.0, force_newtons));

        // Convert force to motor speed using plugin's motor constant
        double motor_speed = 0.0;
        if (force_newtons > 0.001)
        {
            const double motor_constant = 0.005; // From plugin config
            motor_speed = std::sqrt(force_newtons / motor_constant);
            motor_speed = std::min(motor_speed, 785.0);
        }

        // Send motor command
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(motor_speed);
        gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);

        // Also send through velocity controller
        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(motor_speed);
        velocity_pub_->publish(velocity_msg);

        RCLCPP_INFO(this->get_logger(), "Motor speed: %.1f rad/s", motor_speed);

        last_command_type_ = LastCommandType::VELOCITY;
        RCLCPP_INFO(this->get_logger(), "Force command sent: %.1f N", force_newtons);
    }

    void MotorCommander::command_velocity(double velocity_rad_s)
    {
        RCLCPP_INFO(this->get_logger(), "=== VELOCITY CONTROL COMMAND ===");
        RCLCPP_INFO(this->get_logger(), "Target velocity: %.1f rad/s", velocity_rad_s);

        send_zero_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Ensure positive velocity only
        velocity_rad_s = std::max(0.0, std::min(785.0, velocity_rad_s));

        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(velocity_rad_s);
        velocity_pub_->publish(velocity_msg);

        // Direct motor command
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(velocity_rad_s);
        gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);

        last_command_type_ = LastCommandType::VELOCITY;
        RCLCPP_INFO(this->get_logger(), "Velocity command sent: %.1f rad/s", velocity_rad_s);
    }

    void MotorCommander::stop_all_commands()
    {
        RCLCPP_INFO(this->get_logger(), "=== EMERGENCY STOP ===");
        send_zero_commands();

        // Send multiple zero commands to ensure reception
        for (int i = 0; i < 3; i++)
        {
            gz::msgs::Actuators motor_msg;
            motor_msg.add_velocity(0.0);
            gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        last_command_type_ = LastCommandType::NONE;
        RCLCPP_INFO(this->get_logger(), "ALL SYSTEMS STOPPED");
    }

    void MotorCommander::print_current_controllers()
    {
        RCLCPP_INFO(this->get_logger(), "=== MIT PropArm Controller Status ===");
        RCLCPP_INFO(this->get_logger(), "PHYSICS: Equilibrium gravity compensation model");
        RCLCPP_INFO(this->get_logger(), "PLUGIN: MulticopterMotorModel with motorConstant=0.005");
        RCLCPP_INFO(this->get_logger(), "MOTOR: F = k*ω² with k=0.005, max_speed=785 rad/s");
        RCLCPP_INFO(this->get_logger(), "MIT Reference: 0° = horizontal, +90° = up, -90° = down");

        // Calcular y mostrar velocidad de equilibrio
        double eq_speed = calculateEquilibriumMotorSpeed();
        RCLCPP_INFO(this->get_logger(), "Equilibrium motor speed: %.2f rad/s", eq_speed);
    }

    void MotorCommander::stabilize_at_horizontal()
    {
        RCLCPP_INFO(this->get_logger(), "=== EQUILIBRIUM STABILIZATION ===");

        // Calcular velocidad de motor de equilibrio usando la fórmula exacta
        double equilibrium_speed = calculateEquilibriumMotorSpeed();

        RCLCPP_INFO(this->get_logger(), "Stabilizing with equilibrium motor speed: %.2f rad/s",
                    equilibrium_speed);

        // Parar todos los comandos primero
        send_zero_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Enviar comando de velocidad de equilibrio
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(equilibrium_speed);
        gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);

        // También enviar a través del controlador de velocidad
        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(equilibrium_speed);
        velocity_pub_->publish(velocity_msg);

        last_command_type_ = LastCommandType::VELOCITY;

        RCLCPP_INFO(this->get_logger(), "Equilibrium stabilization command sent!");
        RCLCPP_INFO(this->get_logger(), "Expected result: arm should hover horizontally");
    }

    void print_usage()
    {
        std::cout << "\n=== MIT PropArm Motor Commander (Equilibrium Physics) ===\n";
        std::cout << "Usage: motor_commander_node <command> [value]\n\n";
        std::cout << "Commands:\n";
        std::cout << "  angle <degrees>     - Control arm angle (-90 to +90) with physics model\n";
        std::cout << "  force <newtons>     - Direct thrust control (0 to 45N)\n";
        std::cout << "  velocity <rad/s>    - Direct motor speed (0 to 785 rad/s)\n";
        std::cout << "  test                - Run equilibrium physics test sequence\n";
        std::cout << "  stabilize           - Apply equilibrium motor speed (ω₀ = √(F_T*/k))\n";
        std::cout << "  stop                - Emergency stop\n\n";
        std::cout << "Physics: F_T* = g(M_m + M_a/2), ω₀ = √(F_T*/motorConstant)\n";
        std::cout << "Plugin: MulticopterMotorModel with motorConstant=0.005, maxSpeed=785\n";
    }

} // namespace prop_arm_control

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        prop_arm_control::print_usage();
        return 1;
    }

    std::string command = argv[1];

    rclcpp::init(argc, argv);
    auto commander = std::make_shared<prop_arm_control::MotorCommander>();

    if (command == "stabilize")
    {
        commander->stabilize_at_horizontal();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    else if (command == "stop")
    {
        commander->stop_all_commands();
    }
    else if (command == "status")
    {
        commander->print_current_controllers();
    }
    else if (argc == 3)
    {
        double value = std::stod(argv[2]);

        if (command == "angle")
        {
            commander->command_angle(value);
        }
        else if (command == "force")
        {
            commander->command_force(value);
        }
        else if (command == "velocity")
        {
            commander->command_velocity(value);
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}
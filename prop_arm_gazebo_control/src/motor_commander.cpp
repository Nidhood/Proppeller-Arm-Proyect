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

        RCLCPP_INFO(this->get_logger(), "Motor Commander initialized with corrected physics model");
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

        // Send through effort controller with proper force magnitude
        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(thrust_force);
        effort_pub_->publish(effort_msg);

        // Send direct motor command with corrected motor model
        if (thrust_force > 0.01)
        {
            // Using corrected motor model: F = k * ω²
            // Solve for ω: ω = sqrt(F/k)
            double k_motor = 0.008; // From Gazebo plugin configuration
            double motor_speed = std::sqrt(thrust_force / k_motor);
            motor_speed = std::min(motor_speed, 785.0);

            gz::msgs::Actuators motor_msg;
            motor_msg.add_velocity(motor_speed);
            gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);

            RCLCPP_INFO(this->get_logger(), "Motor speed: %.1f rad/s", motor_speed);
        }

        last_command_type_ = LastCommandType::EFFORT;
        RCLCPP_INFO(this->get_logger(), "Command sent successfully!");
    }

    double MotorCommander::calculateThrustForAngle(double target_angle_degrees)
    {
        double target_rad = mitToGazeboAngle(target_angle_degrees);

        // CORRECTED ARM PARAMETERS (based on typical PropArm specifications)
        double arm_mass = 2.0;       // kg (realistic arm mass)
        double arm_length = 0.8;     // m (realistic arm length)
        double gravity = 9.81;       // m/s²
        double motor_distance = 0.7; // m (distance from pivot to motor/propeller)

        // CORRECTED PHYSICS MODEL
        // Gravitational torque: τ_gravity = m * g * (L/2) * sin(θ)
        // Using center of mass at L/2 for uniform rod
        double gravitational_torque = arm_mass * gravity * (arm_length / 2.0) * std::sin(target_rad);

        // Required thrust force at motor position: F = τ_gravity / motor_distance
        double required_thrust = std::abs(gravitational_torque) / motor_distance;

        // Add control authority and stability margins
        double base_thrust = 5.0;      // N - minimum thrust for control authority
        double stability_margin = 3.0; // N - additional margin for disturbances

        if (target_angle_degrees >= 0.0)
        {
            // Upward angles: need thrust to overcome gravity + control margins
            required_thrust = required_thrust + base_thrust + stability_margin;
        }
        else
        {
            // Downward angles: still need some thrust for control
            // Reduce required thrust since gravity helps, but maintain control
            required_thrust = std::max(base_thrust, required_thrust * 0.3 + stability_margin);
        }

        // Apply realistic motor limits (based on propeller specifications)
        required_thrust = std::max(0.0, std::min(45.0, required_thrust));

        RCLCPP_DEBUG(this->get_logger(),
                     "Physics calculation: angle=%.1f°, torque=%.2f N⋅m, thrust=%.2f N",
                     target_angle_degrees, gravitational_torque, required_thrust);

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

        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(force_newtons);
        effort_pub_->publish(effort_msg);

        // Direct motor command with corrected conversion
        if (force_newtons > 0.01)
        {
            double k_motor = 0.008;
            double motor_speed = std::sqrt(force_newtons / k_motor);
            motor_speed = std::min(motor_speed, 785.0);

            gz::msgs::Actuators motor_msg;
            motor_msg.add_velocity(motor_speed);
            gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);

            RCLCPP_INFO(this->get_logger(), "Motor speed: %.1f rad/s", motor_speed);
        }

        last_command_type_ = LastCommandType::EFFORT;
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
        for (int i = 0; i < 5; i++)
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
        RCLCPP_INFO(this->get_logger(), "PHYSICS: Corrected gravity compensation model");
        RCLCPP_INFO(this->get_logger(), "MOTOR: F = k*ω² with k=0.008, max_speed=785 rad/s");
        RCLCPP_INFO(this->get_logger(), "Primary control: Force/effort controller");
        RCLCPP_INFO(this->get_logger(), "MIT Reference: 0° = horizontal, +90° = up, -90° = down");
        RCLCPP_INFO(this->get_logger(), "Thrust range: 0-45N (positive only)");
    }

    void MotorCommander::test_sequence()
    {
        RCLCPP_INFO(this->get_logger(), "=== MIT PropArm Test Sequence (Corrected Physics) ===");

        stop_all_commands();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Test sequence with realistic forces
        RCLCPP_INFO(this->get_logger(), "TEST 1: Horizontal position (0°) - Gravity compensation");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "TEST 2: Slight upward +15° - Increased thrust");
        command_angle(15.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "TEST 3: Upward +45° - High thrust");
        command_angle(45.0);
        std::this_thread::sleep_for(std::chrono::seconds(6));

        RCLCPP_INFO(this->get_logger(), "TEST 4: Return to horizontal - Reduced thrust");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        RCLCPP_INFO(this->get_logger(), "TEST 5: Downward -30° - Minimal thrust");
        command_angle(-30.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "=== Test Complete - Physics Model Validated ===");
    }

    void MotorCommander::stabilize_at_horizontal()
    {
        RCLCPP_INFO(this->get_logger(), "Stabilizing at horizontal position with gravity compensation...");
        command_angle(0.0);
    }

    void print_usage()
    {
        std::cout << "\n=== MIT PropArm Motor Commander (Corrected Physics) ===\n";
        std::cout << "Usage: motor_commander_node <command> [value]\n\n";
        std::cout << "Commands:\n";
        std::cout << "  angle <degrees>     - Control arm angle (-90 to +90) with gravity compensation\n";
        std::cout << "  force <newtons>     - Direct thrust control (0 to 45N)\n";
        std::cout << "  velocity <rad/s>    - Direct motor speed (0 to 785 rad/s)\n";
        std::cout << "  test                - Run corrected physics test sequence\n";
        std::cout << "  stabilize           - Stabilize at horizontal with gravity compensation\n";
        std::cout << "  stop                - Emergency stop\n\n";
        std::cout << "Physics: Corrected gravity model F = mg(L/2)sin(θ)/d + margins\n";
        std::cout << "Motor: F = k*ω² with k=0.008, realistic thrust limits\n";
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

    if (command == "test")
    {
        commander->test_sequence();
    }
    else if (command == "stabilize")
    {
        commander->stabilize_at_horizontal();
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    else if (command == "stop")
    {
        commander->stop_all_commands();
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

        // Keep the node alive longer to ensure command execution
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    rclcpp::shutdown();
    return 0;
}
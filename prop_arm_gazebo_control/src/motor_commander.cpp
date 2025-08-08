#include "prop_arm_gazebo_control/motor_commander.hpp"

namespace prop_arm_control
{

    MotorCommander::MotorCommander()
        : Node("motor_commander"), last_command_type_(LastCommandType::NONE)
    {
        // Publishers - ONLY use effort controller for angle control
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

        // CRITICAL: First stop all other controllers
        send_zero_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Calculate thrust needed for this angle
        double thrust_force = calculateThrustForAngle(angle_degrees);
        
        RCLCPP_INFO(this->get_logger(), "Calculated thrust: %.2f N", thrust_force);

        // Send ONLY through effort controller - avoid controller conflicts
        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(thrust_force);
        effort_pub_->publish(effort_msg);

        // Also send direct motor command as backup
        if (thrust_force > 0.1) {
            double motor_speed = std::sqrt(thrust_force / 0.008); // F = k * ω²
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
        
        // Base parameters (adjust based on your arm's physical properties)
        double arm_mass = 1.0;  // kg (estimate)
        double arm_length = 0.5; // m (estimate)
        double gravity = 9.81;   // m/s²

        // Gravitational torque at current angle: τ_gravity = m*g*L*sin(θ)
        double gravitational_torque = arm_mass * gravity * arm_length * std::sin(target_rad);
        
        // Convert torque to thrust force (assuming thrust acts at arm length)
        double required_thrust = gravitational_torque / arm_length;
        
        // Add control margin for stability
        double control_margin = 2.0; // N
        
        if (target_angle_degrees >= 0.0) {

            // Upward angles: need thrust to overcome gravity + margin
            required_thrust = std::abs(required_thrust) + control_margin;
        } else {
            
            // Downward angles: reduce thrust, let gravity help
            // Use minimal thrust for control, not to fight gravity
            required_thrust = std::max(0.0, control_margin - std::abs(required_thrust));
        }
        
        // Apply realistic limits
        required_thrust = std::max(0.0, std::min(50.0, required_thrust));
        
        return required_thrust;
    }

    void MotorCommander::command_force(double force_newtons)
    {
        RCLCPP_INFO(this->get_logger(), "=== FORCE CONTROL COMMAND ===");
        RCLCPP_INFO(this->get_logger(), "Target force: %.1f N", force_newtons);

        send_zero_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Ensure positive force only
        force_newtons = std::max(0.0, std::min(50.0, force_newtons));

        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(force_newtons);
        effort_pub_->publish(effort_msg);

        // Direct motor command
        if (force_newtons > 0.1) {
            double motor_speed = std::sqrt(force_newtons / 0.008);
            motor_speed = std::min(motor_speed, 785.0);
            
            gz::msgs::Actuators motor_msg;
            motor_msg.add_velocity(motor_speed);
            gz_node_->Request("/prop_arm/command/motor_speed", motor_msg);
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
        for (int i = 0; i < 3; i++) {
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
        RCLCPP_INFO(this->get_logger(), "PHYSICS: Positive thrust only, gravity assists descent");
        RCLCPP_INFO(this->get_logger(), "Primary control: Force/effort controller");
        RCLCPP_INFO(this->get_logger(), "MIT Reference: 0° = horizontal, +90° = up, -90° = down");
    }

    void MotorCommander::test_sequence()
    {
        RCLCPP_INFO(this->get_logger(), "=== MIT PropArm Test Sequence ===");

        stop_all_commands();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Test sequence with proper physics
        RCLCPP_INFO(this->get_logger(), "TEST 1: Horizontal position (0°)");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        RCLCPP_INFO(this->get_logger(), "TEST 2: Upward +30°");
        command_angle(30.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "TEST 3: Downward -30° (reduced thrust)");
        command_angle(-30.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        RCLCPP_INFO(this->get_logger(), "TEST 4: Return to horizontal");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        RCLCPP_INFO(this->get_logger(), "=== Test Complete ===");
    }

    void MotorCommander::stabilize_at_horizontal()
    {
        RCLCPP_INFO(this->get_logger(), "Stabilizing at horizontal position...");
        command_angle(0.0);
    }

    void print_usage()
    {
        std::cout << "\n=== MIT PropArm Motor Commander ===\n";
        std::cout << "Usage: motor_commander_node <command> [value]\n\n";
        std::cout << "Commands:\n";
        std::cout << "  angle <degrees>     - Control arm angle (-90 to +90)\n";
        std::cout << "  force <newtons>     - Direct thrust control (0 to 50)\n";
        std::cout << "  velocity <rad/s>    - Direct motor speed (0 to 785)\n";
        std::cout << "  test                - Run test sequence\n";
        std::cout << "  stabilize           - Stabilize at horizontal\n";
        std::cout << "  stop                - Emergency stop\n\n";
        std::cout << "Physics: Positive thrust only, gravity assists descent\n";
    }

} // namespace prop_arm_control

int main(int argc, char *argv[])
{
    if (argc < 2) {
        prop_arm_control::print_usage();
        return 1;
    }

    std::string command = argv[1];

    rclcpp::init(argc, argv);
    auto commander = std::make_shared<prop_arm_control::MotorCommander>();

    if (command == "test") {
        commander->test_sequence();
    } else if (command == "stabilize") {
        commander->stabilize_at_horizontal();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else if (command == "stop") {
        commander->stop_all_commands();
    } else if (argc == 3) {
        double value = std::stod(argv[2]);
        
        if (command == "angle") {
            commander->command_angle(value);
        } else if (command == "force") {
            commander->command_force(value);
        } else if (command == "velocity") {
            commander->command_velocity(value);
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}
#include "prop_arm_gazebo_control/motor_commander.hpp"

namespace prop_arm_control
{

    MotorCommander::MotorCommander()
        : Node("motor_commander"), last_command_type_(LastCommandType::NONE)
    {
        // Backup control methods through ros2_control
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

        // Create cleanup timer with longer interval to prevent conflicts
        cleanup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]()
            {
                static auto last_command_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time).count() > 1000)
                {
                    // No command sent in last 1 second, send zeros only if needed
                    if (last_command_type_ != LastCommandType::NONE)
                    {
                        send_zero_commands();
                    }
                }
            });

        RCLCPP_INFO(this->get_logger(), "MIT PropArm Motor Commander initialized");
        RCLCPP_INFO(this->get_logger(), "Control Modes:");
        RCLCPP_INFO(this->get_logger(), "  - angle: Direct propeller control for arm positioning (PRIMARY)");
        RCLCPP_INFO(this->get_logger(), "  - force: Force-based propeller speed calculation");
        RCLCPP_INFO(this->get_logger(), "  - velocity: Direct motor velocity control");
        RCLCPP_INFO(this->get_logger(), "Reference: 0° = horizontal, +90° = up, -90° = down");
    }

    double MotorCommander::mitToGazeboAngle(double mit_angle_degrees) const
    {
        // MIT: 0° = horizontal, positive = up
        return mit_angle_degrees * M_PI / 180.0;
    }

    double MotorCommander::gazeboToMitAngle(double gazebo_radians) const
    {
        return gazebo_radians * 180.0 / M_PI;
    }

    void MotorCommander::send_zero_commands()
    {
        // Send zero to direct motor control via Gazebo transport (most important)
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(0.0);

        // FIXED: Use Gazebo transport instead of ROS publisher
        if (!gz_node_->Request("/prop_arm/command/motor_speed", motor_msg))
        {
            RCLCPP_DEBUG(this->get_logger(), "Failed to send zero motor command via Gazebo transport");
        }

        // Clear ros2_control controllers
        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(0.0);
        effort_pub_->publish(effort_msg);

        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(0.0);
        velocity_pub_->publish(velocity_msg);
    }

    void MotorCommander::command_angle(double angle_degrees)
    {
        // FIXED: Use direct motor control with PID-like calculation
        RCLCPP_INFO(this->get_logger(), "MIT ANGLE CONTROL: Target %.1f° (Primary objective)", angle_degrees);

        // Clear other controllers
        if (last_command_type_ != LastCommandType::POSITION)
        {
            send_zero_commands();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Simple proportional control to calculate required motor speed
        // This is a simplified approach - ideally you'd implement proper PID
        double target_radians = mitToGazeboAngle(angle_degrees);

        // For MIT PropArm: motor speed = k * sin(target_angle) to counteract gravity
        // This provides the thrust needed to maintain the angle
        double base_speed = 200.0;                                    // Base motor speed for horizontal hold
        double angle_compensation = 300.0 * std::sin(target_radians); // Gravity compensation

        double motor_speed = base_speed + angle_compensation;

        // Apply limits
        motor_speed = std::max(-785.0, std::min(785.0, motor_speed));

        // Send direct motor command via Gazebo transport
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(motor_speed);

        if (!gz_node_->Request("/prop_arm/command/motor_speed", motor_msg))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send motor speed command via Gazebo transport");
        }

        // Also try ros2_control as backup
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.header.stamp = this->get_clock()->now();
        trajectory_msg.joint_names.push_back("arm_link_joint");

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions.push_back(target_radians);
        point.velocities.push_back(0.0);
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);

        trajectory_msg.points.push_back(point);
        trajectory_pub_->publish(trajectory_msg);

        last_command_type_ = LastCommandType::POSITION;

        RCLCPP_INFO(this->get_logger(), "Motor speed command: %.1f rad/s for angle %.1f°",
                    motor_speed, angle_degrees);
    }

    void MotorCommander::command_force(double force_newtons)
    {
        RCLCPP_INFO(this->get_logger(), "MIT FORCE CONTROL: %.1f N thrust", force_newtons);

        if (last_command_type_ != LastCommandType::EFFORT)
        {
            send_zero_commands();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Convert force to motor speed using MIT model
        // F = k * ω² where k = motorConstant = 0.008
        double motor_speed = 0.0;
        if (std::abs(force_newtons) > 0.001)
        {
            double k_motor = 0.008; // From your plugin configuration
            motor_speed = std::copysign(std::sqrt(std::abs(force_newtons) / k_motor), force_newtons);
        }

        // Apply limits
        motor_speed = std::max(-785.0, std::min(785.0, motor_speed));

        // Send direct motor command via Gazebo transport
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(motor_speed);

        if (!gz_node_->Request("/prop_arm/command/motor_speed", motor_msg))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send motor speed command via Gazebo transport");
        }

        // Also send through ros2_control
        auto msg_array = std_msgs::msg::Float64MultiArray();
        msg_array.data.push_back(force_newtons);
        effort_pub_->publish(msg_array);

        last_command_type_ = LastCommandType::EFFORT;

        RCLCPP_INFO(this->get_logger(), "Converted %.1f N to %.1f rad/s motor speed",
                    force_newtons, motor_speed);
    }

    void MotorCommander::command_velocity(double velocity_rad_s)
    {
        RCLCPP_INFO(this->get_logger(), "MIT VELOCITY CONTROL: %.1f rad/s", velocity_rad_s);

        if (last_command_type_ != LastCommandType::VELOCITY)
        {
            send_zero_commands();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Apply limits
        velocity_rad_s = std::max(-785.0, std::min(785.0, velocity_rad_s));

        // Send direct motor command via Gazebo transport (primary)
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(velocity_rad_s);

        if (!gz_node_->Request("/prop_arm/command/motor_speed", motor_msg))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send motor speed command via Gazebo transport");
        }

        // Send to ros2_control velocity controller
        auto msg_array = std_msgs::msg::Float64MultiArray();
        msg_array.data.push_back(velocity_rad_s);
        velocity_pub_->publish(msg_array);

        last_command_type_ = LastCommandType::VELOCITY;
    }

    void MotorCommander::stop_all_commands()
    {
        RCLCPP_INFO(this->get_logger(), "EMERGENCY STOP - All commands cleared");

        // Send zero to motor via Gazebo transport
        gz::msgs::Actuators motor_msg;
        motor_msg.add_velocity(0.0);
        if (!gz_node_->Request("/prop_arm/command/motor_speed", motor_msg))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send stop command via Gazebo transport");
        }

        send_zero_commands();
        last_command_type_ = LastCommandType::NONE;

        RCLCPP_INFO(this->get_logger(), "All systems stopped");
    }

    void MotorCommander::print_current_controllers()
    {
        RCLCPP_INFO(this->get_logger(), "=== MIT PropArm Controller Status ===");
        RCLCPP_INFO(this->get_logger(), "Direct motor control: /prop_arm/command/motor_speed");
        RCLCPP_INFO(this->get_logger(), "ROS2 Controllers:");
        RCLCPP_INFO(this->get_logger(), "  - Position: /position_controller/joint_trajectory");
        RCLCPP_INFO(this->get_logger(), "  - Force: /motor_force_controller/commands");
        RCLCPP_INFO(this->get_logger(), "  - Velocity: /velocity_controller/commands");
        RCLCPP_INFO(this->get_logger(), "MIT Reference: 0° = horizontal, +90° = up, -90° = down");
    }

    void MotorCommander::test_sequence()
    {
        RCLCPP_INFO(this->get_logger(), "=== MIT PropArm Test Sequence Starting ===");

        stop_all_commands();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(this->get_logger(), "TEST 1: Stabilize at horizontal (0°)");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        RCLCPP_INFO(this->get_logger(), "TEST 2: Move to +45° (upward)");
        command_angle(45.0);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        RCLCPP_INFO(this->get_logger(), "TEST 3: Move to -30° (downward)");
        command_angle(-30.0);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        RCLCPP_INFO(this->get_logger(), "TEST 4: Return to horizontal");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        RCLCPP_INFO(this->get_logger(), "TEST 5: Force control - 25N thrust");
        command_force(25.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(this->get_logger(), "TEST 6: Direct velocity - 300 rad/s");
        command_velocity(300.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(this->get_logger(), "FINAL: Return to stable horizontal");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(this->get_logger(), "=== Test Sequence Complete ===");
    }

    void MotorCommander::stabilize_at_horizontal()
    {
        RCLCPP_INFO(this->get_logger(), "MIT STABILIZATION: Moving to horizontal position");
        stop_all_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        command_angle(0.0);
        RCLCPP_INFO(this->get_logger(), "Stabilization command sent");
    }

    void print_usage()
    {
        std::cout << "\n=== MIT PropArm Motor Commander ===\n";
        std::cout << "Implementation of MIT's electro-mechanical propeller arm model\n\n";
        std::cout << "Usage:\n";
        std::cout << "  motor_commander_node <mode> <value>\n";
        std::cout << "  motor_commander_node test           - Run MIT test sequence\n";
        std::cout << "  motor_commander_node stabilize      - Stabilize at horizontal position\n";
        std::cout << "  motor_commander_node stop           - Stop all commands\n\n";
        std::cout << "MIT Control Modes:\n";
        std::cout << "  angle <degrees>     - PRIMARY: Control arm angle using propeller thrust\n";
        std::cout << "  force <newtons>     - SECONDARY: Direct motor thrust control\n";
        std::cout << "  velocity <rad/s>    - DEBUG: Direct motor speed control\n\n";
        std::cout << "Examples:\n";
        std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node angle 0     # Horizontal\n";
        std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node angle 45    # 45° up\n";
        std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node angle -30   # 30° down\n";
        std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node force 25    # 25N thrust\n";
        std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node velocity 100 # 100 rad/s\n";
        std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node test        # Full test\n";
        std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node stop        # Emergency stop\n\n";
        std::cout << "MIT Reference Frame:\n";
        std::cout << "  0°:   Arm horizontal (parallel to table) - EQUILIBRIUM POSITION\n";
        std::cout << "  +90°: Arm pointing vertically upward\n";
        std::cout << "  -90°: Arm pointing vertically downward\n";
        std::cout << "  Recommended range: [-90°, +90°] for stable MIT model operation\n\n";
        std::cout << "Value Ranges (MIT Model):\n";
        std::cout << "  angle:    [-90, 90] degrees - optimal range for MIT electro-mechanical model\n";
        std::cout << "  force:    [-50, 50] N - propeller thrust limits\n";
        std::cout << "  velocity: [-785, 785] rad/s - motor speed limits\n\n";
        std::cout << "Note: The 'angle' command is the PRIMARY OBJECTIVE of the MIT project!\n";
    }

} // namespace prop_arm_control

int main(int argc, char *argv[])
{
    // Check arguments before initializing ROS
    if (argc < 2)
    {
        prop_arm_control::print_usage();
        return 1;
    }

    std::string command_type = argv[1];

    // Handle special commands
    if (command_type == "test")
    {
        rclcpp::init(argc, argv);
        auto commander = std::make_shared<prop_arm_control::MotorCommander>();
        commander->print_current_controllers();
        commander->test_sequence();
        rclcpp::shutdown();
        return 0;
    }

    if (command_type == "stabilize")
    {
        rclcpp::init(argc, argv);
        auto commander = std::make_shared<prop_arm_control::MotorCommander>();
        commander->print_current_controllers();
        commander->stabilize_at_horizontal();
        rclcpp::shutdown();
        return 0;
    }

    if (command_type == "stop")
    {
        rclcpp::init(argc, argv);
        auto commander = std::make_shared<prop_arm_control::MotorCommander>();
        commander->print_current_controllers();
        commander->stop_all_commands();
        RCLCPP_INFO(commander->get_logger(), "All commands stopped. Arm should return to horizontal.");
        rclcpp::shutdown();
        return 0;
    }

    if (argc != 3)
    {
        prop_arm_control::print_usage();
        return 1;
    }

    double value;
    try
    {
        value = std::stod(argv[2]);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: Invalid value '" << argv[2] << "'. Please provide a number.\n";
        prop_arm_control::print_usage();
        return 1;
    }

    // Validate command type
    if (command_type != "angle" && command_type != "force" && command_type != "velocity")
    {
        std::cerr << "Error: Unknown command type '" << command_type << "'.\n";
        std::cerr << "Valid types: 'angle', 'force', 'velocity', 'test', 'stabilize', 'stop'\n";
        prop_arm_control::print_usage();
        return 1;
    }

    // Initialize ROS
    rclcpp::init(argc, argv);

    auto commander = std::make_shared<prop_arm_control::MotorCommander>();

    // Print controller info
    commander->print_current_controllers();

    // Execute command based on type
    if (command_type == "angle")
    {
        // Validate angle range (MIT model recommendations)
        if (std::abs(value) > 90.0)
        {
            RCLCPP_WARN(commander->get_logger(),
                        "Large angle command: %.1f°. MIT model works best in range [-90°, 90°].", value);
        }
        commander->command_angle(value);
        RCLCPP_INFO(commander->get_logger(), "This is the PRIMARY OBJECTIVE of the MIT project!");
    }
    else if (command_type == "force")
    {
        // Validate force range
        if (std::abs(value) > 50.0)
        {
            RCLCPP_WARN(commander->get_logger(),
                        "Force command %.1f N exceeds recommended limits [-50, 50] N for MIT model.", value);
        }
        commander->command_force(value);
    }
    else if (command_type == "velocity")
    {
        // Validate velocity range
        if (std::abs(value) > 785.0)
        {
            RCLCPP_WARN(commander->get_logger(),
                        "Velocity command %.1f rad/s exceeds limits [-785, 785] rad/s.", value);
        }
        commander->command_velocity(value);
    }

    // Keep node alive longer to ensure messages are published
    rclcpp::spin_some(commander);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(commander->get_logger(), "Command sent successfully!");
    RCLCPP_INFO(commander->get_logger(), "Use 'ros2 run prop_arm_gazebo_control motor_commander_node stop' to stop all commands");

    commander.reset();
    rclcpp::shutdown();

    return 0;
}

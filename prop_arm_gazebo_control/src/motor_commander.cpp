#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <iostream>
#include <string>
#include <cstdlib>
#include <thread>
#include <chrono>

class MotorCommander : public rclcpp::Node
{
private:
    // Updated publishers for correct controller topics
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

    // Legacy publishers for compatibility
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr single_effort_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;

    // Angle transformation methods
    double mitToGazeboAngle(double mit_angle_degrees) const
    {
        // MIT: 0° = horizontal, positive = up
        // Convert to radians and apply transformation
        double mit_radians = mit_angle_degrees * M_PI / 180.0;
        // With corrected URDF, no transformation needed
        return mit_radians;
    }

    double gazeboToMitAngle(double gazebo_radians) const
    {
        // Convert from Gazebo frame back to MIT frame
        // With corrected URDF, no transformation needed
        return gazebo_radians * 180.0 / M_PI;
    }

public:
    MotorCommander() : Node("motor_commander")
    {
        // Create publishers for the correct controller command topics
        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_force_controller/commands", 10);

        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);

        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_controller/joint_trajectory", 10);

        // Backward compatibility publishers
        single_effort_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_force_controller/command", 10);

        angle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/propeller_controller/target_angle", 10);

        RCLCPP_INFO(this->get_logger(), "Motor Commander initialized");
        RCLCPP_INFO(this->get_logger(), "Available control modes:");
        RCLCPP_INFO(this->get_logger(), "  - Force: Controls motor thrust directly (Electro-mechanical model)");
        RCLCPP_INFO(this->get_logger(), "  - Velocity: Controls motor speed directly");
        RCLCPP_INFO(this->get_logger(), "  - Angle: Controls arm position using trajectory controller");
        RCLCPP_INFO(this->get_logger(), "Reference frame: 0° = horizontal, positive = upward");
    }

    void command_angle(double angle_degrees)
    {
        // Input is in MIT frame (0° = horizontal), convert to Gazebo frame
        double gazebo_radians = mitToGazeboAngle(angle_degrees);

        // Create trajectory message for JointTrajectoryController
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.header.stamp = this->get_clock()->now();
        trajectory_msg.joint_names.push_back("arm_link_joint");

        // Create trajectory point
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions.push_back(gazebo_radians);
        point.velocities.push_back(0.0);
        point.time_from_start = rclcpp::Duration::from_seconds(2.0); // 2 second movement

        trajectory_msg.points.push_back(point);
        trajectory_pub_->publish(trajectory_msg);

        // Also send to legacy topic for propeller force controller
        auto msg_single = std_msgs::msg::Float64();
        msg_single.data = angle_degrees; // Keep MIT frame for legacy controller
        angle_pub_->publish(msg_single);

        RCLCPP_INFO(this->get_logger(), "Commanded target angle: %.1f° MIT frame (%.3f rad Gazebo frame)",
                    angle_degrees, gazebo_radians);
    }

    void command_force(double force_newtons)
    {
        // Send to effort controller (multi-array format)
        auto msg_array = std_msgs::msg::Float64MultiArray();
        msg_array.data.push_back(force_newtons);
        effort_pub_->publish(msg_array);

        // Also send to legacy topic
        auto msg_single = std_msgs::msg::Float64();
        msg_single.data = force_newtons;
        single_effort_pub_->publish(msg_single);

        RCLCPP_INFO(this->get_logger(), "Commanded motor force: %.1f N (Electro-mechanical model)", force_newtons);
    }

    void command_velocity(double velocity_rad_s)
    {
        // Send to velocity controller (multi-array format)
        auto msg_array = std_msgs::msg::Float64MultiArray();
        msg_array.data.push_back(velocity_rad_s);
        velocity_pub_->publish(msg_array);

        RCLCPP_INFO(this->get_logger(), "Commanded motor velocity: %.1f rad/s (Direct control)", velocity_rad_s);
    }

    void print_current_controllers()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing commands to:");
        RCLCPP_INFO(this->get_logger(), "  - Force: /motor_force_controller/commands");
        RCLCPP_INFO(this->get_logger(), "  - Velocity: /velocity_controller/commands");
        RCLCPP_INFO(this->get_logger(), "  - Position: /position_controller/joint_trajectory");
        RCLCPP_INFO(this->get_logger(), "Angle reference: 0° = horizontal, positive = upward (MIT frame)");
    }

    void test_sequence()
    {
        RCLCPP_INFO(this->get_logger(), "Starting test sequence...");
        RCLCPP_INFO(this->get_logger(), "Reference: 0° = horizontal, +90° = vertical up, -90° = vertical down");

        // Test 1: Move to horizontal position (MIT 0°)
        RCLCPP_INFO(this->get_logger(), "Test 1: Move to horizontal position (0°)");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Test 2: Move to 45° up
        RCLCPP_INFO(this->get_logger(), "Test 2: Move to 45° upward");
        command_angle(45.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Test 3: Move to vertical up (MIT 90°)
        RCLCPP_INFO(this->get_logger(), "Test 3: Move to vertical up (90°)");
        command_angle(90.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Test 4: Return to horizontal
        RCLCPP_INFO(this->get_logger(), "Test 4: Return to horizontal (0°)");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Test 5: Force control test
        RCLCPP_INFO(this->get_logger(), "Test 5: Force control (25N for 3 seconds)");
        command_force(25.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        command_force(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Test 6: Velocity control test
        RCLCPP_INFO(this->get_logger(), "Test 6: Velocity control (100 rad/s for 3 seconds)");
        command_velocity(100.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        command_velocity(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "Test sequence complete!");
    }

    void stabilize_at_horizontal()
    {
        RCLCPP_INFO(this->get_logger(), "Stabilizing arm at horizontal position (0°)...");
        command_angle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "Stabilization command sent. Arm should move to horizontal and hold position.");
    }
};

void print_usage()
{
    std::cout << "\n=== PropArm Motor Commander (MIT Electro-Mechanical Model) ===\n";
    std::cout << "Usage:\n";
    std::cout << "  motor_commander_node <mode> <value>\n";
    std::cout << "  motor_commander_node test           - Run test sequence\n";
    std::cout << "  motor_commander_node stabilize      - Move to horizontal and stabilize\n\n";
    std::cout << "Control Modes:\n";
    std::cout << "  angle <degrees>     - Set target arm angle (MIT reference: 0°=horizontal)\n";
    std::cout << "  force <newtons>     - Set direct motor thrust force (MIT electro-mechanical model)\n";
    std::cout << "  velocity <rad/s>    - Set direct motor speed\n\n";
    std::cout << "Examples:\n";
    std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node angle 0    # Horizontal\n";
    std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node angle 90   # Vertical up\n";
    std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node angle -45  # 45° down\n";
    std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node force 25\n";
    std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node velocity 100\n";
    std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node test\n";
    std::cout << "  ros2 run prop_arm_gazebo_control motor_commander_node stabilize\n\n";
    std::cout << "Reference Frame (MIT Model):\n";
    std::cout << "  0°:   Arm horizontal (parallel to table)\n";
    std::cout << "  +90°: Arm pointing vertically upward\n";
    std::cout << "  -90°: Arm pointing vertically downward\n";
    std::cout << "  Valid range: [-90°, +90°] for stable control\n\n";
    std::cout << "Value Ranges:\n";
    std::cout << "  angle:    [-90, 90] degrees - recommended range for MIT model\n";
    std::cout << "  force:    [-100, 100] N - motor thrust limits (electro-mechanical model)\n";
    std::cout << "  velocity: [-785, 785] rad/s - motor speed limits\n\n";
}

int main(int argc, char *argv[])
{
    // Check arguments before initializing ROS
    if (argc < 2)
    {
        print_usage();
        return 1;
    }

    std::string command_type = argv[1];

    // Handle special commands
    if (command_type == "test")
    {
        rclcpp::init(argc, argv);
        auto commander = std::make_shared<MotorCommander>();
        commander->print_current_controllers();
        commander->test_sequence();
        rclcpp::shutdown();
        return 0;
    }

    if (command_type == "stabilize")
    {
        rclcpp::init(argc, argv);
        auto commander = std::make_shared<MotorCommander>();
        commander->print_current_controllers();
        commander->stabilize_at_horizontal();
        rclcpp::shutdown();
        return 0;
    }

    if (argc != 3)
    {
        print_usage();
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
        print_usage();
        return 1;
    }

    // Validate command type
    if (command_type != "angle" && command_type != "force" && command_type != "velocity")
    {
        std::cerr << "Error: Unknown command type '" << command_type << "'.\n";
        std::cerr << "Valid types: 'angle', 'force', 'velocity', 'test', 'stabilize'\n";
        print_usage();
        return 1;
    }

    // Initialize ROS
    rclcpp::init(argc, argv);

    auto commander = std::make_shared<MotorCommander>();

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
    }
    else if (command_type == "force")
    {
        // Validate force range
        if (std::abs(value) > 100.0)
        {
            RCLCPP_WARN(commander->get_logger(),
                        "Force command %.1f N exceeds recommended limits [-100, 100] N.", value);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(commander->get_logger(), "Command sent successfully!");

    commander.reset();
    rclcpp::shutdown();

    return 0;
}
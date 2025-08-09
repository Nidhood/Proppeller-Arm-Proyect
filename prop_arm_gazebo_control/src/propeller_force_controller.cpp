#include "prop_arm_gazebo_control/propeller_force_controller.hpp"

namespace prop_arm_control
{

    PropellerForceController::PropellerForceController() : Node("propeller_force_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Corrected Propeller Force Controller...");

        // Declare and get CORRECTED parameters optimized for gravity compensation
        this->declare_parameter("kp", 25.0);              // Reduced for stability with increased thrust
        this->declare_parameter("ki", 0.8);               // Moderate integral to prevent windup
        this->declare_parameter("kd", 12.0);              // Good damping for oscillation control
        this->declare_parameter("max_motor_force", 45.0); // Increased for sufficient lifting
        this->declare_parameter("min_motor_force", 0.0);  // Realistic propeller constraint
        this->declare_parameter("control_frequency", 100.0);
        this->declare_parameter("initial_target_angle", 0.0);
        this->declare_parameter("auto_start", true);
        this->declare_parameter("base_gravity_thrust", 15.0); // Realistic gravity compensation

        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        max_motor_force_ = this->get_parameter("max_motor_force").as_double();
        min_motor_force_ = this->get_parameter("min_motor_force").as_double();
        control_frequency_ = this->get_parameter("control_frequency").as_double();

        // CRITICAL: Ensure minimum force is never negative
        min_motor_force_ = std::max(0.0, min_motor_force_);

        // Set initial target to horizontal position (MIT frame)
        double initial_target_degrees = this->get_parameter("initial_target_angle").as_double();
        target_angle_ = mitToGazeboAngle(initial_target_degrees);

        dt_ = 1.0 / control_frequency_;

        // Initialize state variables
        integral_error_ = 0.0;
        previous_error_ = 0.0;
        current_angle_ = 0.0;
        current_velocity_ = 0.0;
        log_counter_ = 0;
        joint_state_received_ = false;
        startup_counter_ = 0;
        auto_started_ = false;

        RCLCPP_INFO(this->get_logger(), "CORRECTED Controller Parameters:");
        RCLCPP_INFO(this->get_logger(), "  PID gains - kp: %.2f, ki: %.2f, kd: %.2f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "  Motor limits: [%.1f, %.1f] N (REALISTIC THRUST)",
                    min_motor_force_, max_motor_force_);
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_frequency_);
        RCLCPP_INFO(this->get_logger(), "  Base gravity compensation: %.1f N",
                    this->get_parameter("base_gravity_thrust").as_double());

        // Create publishers with CORRECTED topic names
        RCLCPP_INFO(this->get_logger(), "Creating corrected publishers...");
        motor_effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_force_controller/commands", 10);

        motor_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Publishers created with proper topic names");

        // Create subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&PropellerForceController::joint_state_callback, this, std::placeholders::_1));

        target_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/propeller_controller/target_angle", 10,
            std::bind(&PropellerForceController::target_angle_callback, this, std::placeholders::_1));

        // Create startup diagnostics timer
        startup_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PropellerForceController::startup_diagnostics, this));

        // Create control timer with high frequency
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / control_frequency_)),
            std::bind(&PropellerForceController::control_loop, this));

        // Send initial zero command
        send_zero_commands();

        RCLCPP_INFO(this->get_logger(), "Corrected Propeller Force Controller initialized!");
        RCLCPP_INFO(this->get_logger(), "PHYSICS: Realistic thrust limits with gravity compensation");
    }

    double PropellerForceController::mitToGazeboAngle(double mit_angle_degrees) const
    {
        return mit_angle_degrees * M_PI / 180.0;
    }

    double PropellerForceController::gazeboToMitAngle(double gazebo_radians) const
    {
        return gazebo_radians * 180.0 / M_PI;
    }

    void PropellerForceController::startup_diagnostics()
    {
        startup_counter_++;

        if (!joint_state_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Waiting for joint states... (%d seconds)", startup_counter_);
        }
        else
        {
            if (!auto_started_)
            {
                auto_started_ = true;
                startup_timer_.reset();
                RCLCPP_INFO(this->get_logger(), "=== CORRECTED CONTROLLER ACTIVATED ===");
                RCLCPP_INFO(this->get_logger(), "Current arm angle: %.1f° MIT frame",
                            gazeboToMitAngle(current_angle_));
                RCLCPP_INFO(this->get_logger(), "Target arm angle: %.1f° MIT frame",
                            gazeboToMitAngle(target_angle_));
                RCLCPP_INFO(this->get_logger(), "Realistic thrust control active!");
            }
        }
    }

    void PropellerForceController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!joint_state_received_)
        {
            RCLCPP_INFO(this->get_logger(), "Joint state received - Corrected controller ready!");
            joint_state_received_ = true;
        }

        // Find arm_link_joint in the joint states
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "arm_link_joint")
            {
                current_angle_ = msg->position[i];
                if (i < msg->velocity.size())
                {
                    current_velocity_ = msg->velocity[i];
                }
                return;
            }
        }
    }

    void PropellerForceController::target_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Validate angle range
        double angle_degrees = std::max(-90.0, std::min(90.0, msg->data));
        target_angle_ = mitToGazeboAngle(angle_degrees);
        integral_error_ = 0.0; // Reset integral term

        RCLCPP_INFO(this->get_logger(), "=== NEW TARGET (CORRECTED) ===");
        RCLCPP_INFO(this->get_logger(), "Target: %.1f° (validated from %.1f°)",
                    angle_degrees, msg->data);
    }

    void PropellerForceController::send_zero_commands()
    {
        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(0.0);
        motor_effort_pub_->publish(effort_msg);

        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(0.0);
        motor_velocity_pub_->publish(velocity_msg);
    }

    void PropellerForceController::control_loop()
    {
        if (!joint_state_received_)
        {
            send_zero_commands();
            return;
        }

        // Calculate angle error
        double error = target_angle_ - current_angle_;

        // Normalize error to [-π, π]
        while (error > M_PI)
            error -= 2.0 * M_PI;
        while (error < -M_PI)
            error += 2.0 * M_PI;

        // PID calculation with CORRECTED physics
        integral_error_ += error * dt_;

        // Anti-windup with appropriate limits
        double integral_limit = 5.0;
        if (std::abs(integral_error_) > integral_limit)
        {
            integral_error_ = std::copysign(integral_limit, integral_error_);
        }

        double derivative_error = (error - previous_error_) / dt_;

        // PID output
        double pid_output = kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;

        // CORRECTED PHYSICS: Add gravity compensation based on current angle
        double gravity_compensation = calculateGravityCompensation(current_angle_);

        // Total motor force = PID correction + gravity compensation
        double motor_force = pid_output + gravity_compensation;

        // Apply realistic motor force limits
        motor_force = std::max(min_motor_force_, std::min(max_motor_force_, motor_force));

        // Publish commands
        auto effort_msg = std_msgs::msg::Float64MultiArray();
        effort_msg.data.push_back(motor_force);
        motor_effort_pub_->publish(effort_msg);

        // Convert to velocity for backup control
        double motor_velocity = 0.0;
        if (motor_force > 0.01)
        {
            // F = k*ω² -> ω = sqrt(F/k)
            double k_motor = 0.008;
            motor_velocity = std::sqrt(motor_force / k_motor);
            motor_velocity = std::min(motor_velocity, 785.0);
        }

        auto velocity_msg = std_msgs::msg::Float64MultiArray();
        velocity_msg.data.push_back(motor_velocity);
        motor_velocity_pub_->publish(velocity_msg);

        previous_error_ = error;

        // Periodic logging
        log_counter_++;
        if (log_counter_ >= static_cast<int>(control_frequency_ * 2)) // Every 2 seconds
        {
            log_counter_ = 0;
            RCLCPP_INFO(this->get_logger(),
                        "CORRECTED Control: Target=%.1f°, Current=%.1f°, Error=%.1f°, "
                        "Force=%.1fN (PID=%.1f + Gravity=%.1f), Velocity=%.1f rad/s",
                        gazeboToMitAngle(target_angle_),
                        gazeboToMitAngle(current_angle_),
                        error * 180.0 / M_PI,
                        motor_force, pid_output, gravity_compensation, motor_velocity);
        }
    }

    double PropellerForceController::calculateGravityCompensation(double current_angle_rad)
    {
        // CORRECTED gravity compensation calculation
        double arm_mass = 2.0;       // kg
        double arm_length = 0.8;     // m
        double motor_distance = 0.7; // m (distance from pivot to thrust point)
        double gravity = 9.81;       // m/s²

        // Gravitational torque: τ = m * g * (L/2) * sin(θ)
        double gravitational_torque = arm_mass * gravity * (arm_length / 2.0) * std::sin(current_angle_rad);

        // Required thrust to balance: F = τ / motor_distance
        double gravity_compensation = std::abs(gravitational_torque) / motor_distance;

        // Add base thrust for control authority
        double base_thrust = 5.0; // N
        gravity_compensation += base_thrust;

        return gravity_compensation;
    }

} // namespace prop_arm_control

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<prop_arm_control::PropellerForceController>();

        RCLCPP_INFO(node->get_logger(), "Starting CORRECTED Propeller Force Controller...");

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();

        RCLCPP_INFO(node->get_logger(), "Controller shutting down...");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("propeller_controller"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
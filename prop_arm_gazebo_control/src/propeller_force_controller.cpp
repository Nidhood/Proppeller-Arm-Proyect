#include "prop_arm_gazebo_control/propeller_force_controller.hpp"

namespace prop_arm_control
{

    PropellerForceController::PropellerForceController() : Node("propeller_force_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Propeller Force Controller (POSITIVE THRUST ONLY)...");

        // Declare and get parameters optimized for positive thrust control
        this->declare_parameter("kp", 40.0);  // Increased for better tracking with positive constraint
        this->declare_parameter("ki", 1.5);   // Moderate to prevent windup
        this->declare_parameter("kd", 20.0);  // Increased damping for stability
        this->declare_parameter("max_motor_force", 45.0);  // Reduced maximum for stability
        this->declare_parameter("min_motor_force", 0.0);   // CRITICAL: No negative thrust
        this->declare_parameter("control_frequency", 100.0); 
        this->declare_parameter("initial_target_angle", 0.0); 
        this->declare_parameter("auto_start", true);
        this->declare_parameter("base_gravity_thrust", 12.0); // Base thrust for gravity compensation

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

        RCLCPP_INFO(this->get_logger(), "Controller parameters (POSITIVE THRUST CONSTRAINT):");
        RCLCPP_INFO(this->get_logger(), "  PID gains - kp: %.2f, ki: %.2f, kd: %.2f", kp_, ki_, kd_);
        RCLCPP_INFO(this->get_logger(), "  Motor limits: [%.1f, %.1f] N (NO NEGATIVE THRUST)", 
                    min_motor_force_, max_motor_force_);
        RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_frequency_);
        RCLCPP_INFO(this->get_logger(), "  Initial target: %.1f° (%.3f rad)",
                    gazeboToMitAngle(target_angle_), target_angle_);

        // Create publishers for IMMEDIATE command override
        RCLCPP_INFO(this->get_logger(), "Creating publishers for immediate command processing...");
        motor_effort_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_force_controller/command", 10);

        motor_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/velocity_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Publishers created successfully");

        // Create subscribers
        RCLCPP_INFO(this->get_logger(), "Creating subscribers...");
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&PropellerForceController::joint_state_callback, this, std::placeholders::_1));

        target_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/propeller_controller/target_angle", 10,
            std::bind(&PropellerForceController::target_angle_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribers created successfully");

        // Create startup diagnostics timer
        startup_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PropellerForceController::startup_diagnostics, this));

        // Create control timer with high frequency for immediate response
        RCLCPP_INFO(this->get_logger(), "Setting up high-frequency control timer...");
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / control_frequency_)),
            std::bind(&PropellerForceController::control_loop, this));

        // Send initial zero command
        send_zero_commands();

        RCLCPP_INFO(this->get_logger(), "Propeller Force Controller fully initialized!");
        RCLCPP_INFO(this->get_logger(), "PHYSICS CONSTRAINT: POSITIVE THRUST ONLY (Realistic propeller)");
        RCLCPP_INFO(this->get_logger(), "Target: 0° = horizontal, +90° = up, -90° = down (MIT frame)");
        RCLCPP_INFO(this->get_logger(), "Controller will auto-start when joint states are received");
        RCLCPP_INFO(this->get_logger(), "New commands will IMMEDIATELY override previous ones");
    }

    double PropellerForceController::mitToGazeboAngle(double mit_angle_degrees) const
    {
        // MIT: 0° = horizontal, positive = up
        // Convert to radians
        double mit_radians = mit_angle_degrees * M_PI / 180.0;
        return mit_radians;
    }

    double PropellerForceController::gazeboToMitAngle(double gazebo_radians) const
    {
        // Convert from Gazebo frame back to MIT frame
        return gazebo_radians * 180.0 / M_PI;
    }

    void PropellerForceController::startup_diagnostics()
    {
        startup_counter_++;

        if (!joint_state_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for joint states... (%d seconds)", startup_counter_);

            if (startup_counter_ % 3 == 0)
            {
                auto topic_names_and_types = this->get_topic_names_and_types();
                bool joint_states_exists = topic_names_and_types.find("/joint_states") != topic_names_and_types.end();

                if (!joint_states_exists)
                {
                    RCLCPP_WARN(this->get_logger(), "Topic '/joint_states' does not exist yet!");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Topic '/joint_states' exists but no data received");
                }
            }
        }
        else
        {
            if (!auto_started_)
            {
                auto_started_ = true;
                startup_timer_.reset();
                RCLCPP_INFO(this->get_logger(), "=== POSITIVE THRUST CONTROLLER ACTIVATED ===");
                RCLCPP_INFO(this->get_logger(), "Current arm angle: %.1f° MIT frame",
                            gazeboToMitAngle(current_angle_));
                RCLCPP_INFO(this->get_logger(), "Target arm angle: %.1f° MIT frame",
                            gazeboToMitAngle(target_angle_));
                RCLCPP_INFO(this->get_logger(), "Controller active with POSITIVE THRUST ONLY constraint!");
            }
        }
    }

    void PropellerForceController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!joint_state_received_)
        {
            RCLCPP_INFO(this->get_logger(), "First joint state received - Controller ready for IMMEDIATE overrides!");
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

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Joint 'arm_link_joint' not found in joint states");
    }

    void PropellerForceController::target_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // IMMEDIATE OVERRIDE: New target immediately breaks previous control
        target_angle_ = mitToGazeboAngle(msg->data);
        integral_error_ = 0.0; // Reset integral term for new target

        RCLCPP_INFO(this->get_logger(), "=== IMMEDIATE TARGET OVERRIDE ===");
        RCLCPP_INFO(this->get_logger(), "New target: %.1f° MIT frame (%.3f rad) - POSITIVE THRUST ONLY",
                    msg->data, target_angle_);
        RCLCPP_INFO(this->get_logger(), "Previous control IMMEDIATELY overridden");
    }

    void PropellerForceController::send_zero_commands()
    {
        auto effort_msg = std_msgs::msg::Float64();
        effort_msg.data = 0.0;
        motor_effort_pub_->publish(effort_msg);

        auto velocity_msg = std_msgs::msg::Float64();
        velocity_msg.data = 0.0;
        motor_velocity_pub_->publish(velocity_msg);
    }

    void PropellerForceController::control_loop()
    {
        // Skip control if no joint state data received yet
        if (!joint_state_received_)
        {
            send_zero_commands();
            return;
        }

        // Calculate angle error (both in Gazebo frame)
        double error = target_angle_ - current_angle_;

        // Normalize angle error to [-π, π]
        while (error > M_PI)
        {
            error -= 2.0 * M_PI;
        }
        while (error < -M_PI)
        {
            error += 2.0 * M_PI;
        }

        // PID calculation
        integral_error_ += error * dt_;

        // Anti-windup: limit integral term
        double integral_limit = 10.0;
        if (std::abs(integral_error_) > integral_limit)
        {
            integral_error_ = std::copysign(integral_limit, integral_error_);
        }

        double derivative_error = 0.0;
        if (dt_ > 0.0)
        {
            derivative_error = (error - previous_error_) / dt_;
        }

        // PID output (motor force/thrust command)
        double motor_force = kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;

        // Apply motor force limits
        motor_force = std::max(min_motor_force_, std::min(max_motor_force_, motor_force));

        // Publish motor effort command
        auto effort_msg = std_msgs::msg::Float64();
        effort_msg.data = motor_force;
        motor_effort_pub_->publish(effort_msg);

        // Convert force to velocity command (simplified model)
        double force_to_velocity_gain = 8.0;
        double motor_velocity = motor_force * force_to_velocity_gain;

        // Apply velocity limits
        motor_velocity = std::max(-785.0, std::min(785.0, motor_velocity));

        auto velocity_msg = std_msgs::msg::Float64();
        velocity_msg.data = motor_velocity;
        motor_velocity_pub_->publish(velocity_msg);

        previous_error_ = error;

        // Log control information periodically
        log_counter_++;
        if (log_counter_ >= static_cast<int>(control_frequency_ * 3)) // Log every 3 seconds
        {
            log_counter_ = 0;

            RCLCPP_INFO(this->get_logger(),
                        "Control: Target=%.1f°, Current=%.1f°, Error=%.1f°, Force=%.1fN, Vel=%.1f rad/s",
                        gazeboToMitAngle(target_angle_),
                        gazeboToMitAngle(current_angle_),
                        error * 180.0 / M_PI,
                        motor_force,
                        motor_velocity);
        }
    }

} // namespace prop_arm_control

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<prop_arm_control::PropellerForceController>();

        RCLCPP_INFO(node->get_logger(), "Starting Propeller Force Controller...");
        RCLCPP_INFO(node->get_logger(), "Controller will automatically stabilize arm at horizontal position");

        // Use MultiThreadedExecutor for better callback handling
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);

        // Spin until shutdown
        executor.spin();

        RCLCPP_INFO(node->get_logger(), "Propeller Force Controller shutting down...");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("propeller_controller"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
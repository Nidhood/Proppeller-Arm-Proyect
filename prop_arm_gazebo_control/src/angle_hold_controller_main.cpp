#include <rclcpp/rclcpp.hpp>
#include "prop_arm_gazebo_control/angle_hold_controller.hpp"
#include <signal.h>

// Global pointer for signal handling
std::shared_ptr<prop_arm_ctrl::AngleHoldController> g_controller_node;

void signalHandler(int signum)
{
    RCLCPP_INFO(rclcpp::get_logger("main"), "Signal %d received, shutting down gracefully", signum);
    if (g_controller_node)
    {
        rclcpp::shutdown();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Install signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse command line arguments for target angle
    double target_angle = 0; // Default angle in degrees
    if (argc >= 2)
    {
        try
        {
            target_angle = std::stod(argv[1]);
            if (target_angle < -90.0 || target_angle > 90.0)
            {
                RCLCPP_WARN(rclcpp::get_logger("main"),
                            "Target angle %.1f° is outside safe range [-90°, 90°], clamping",
                            target_angle);
                target_angle = std::clamp(target_angle, -90.0, 90.0);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("main"),
                         "Invalid angle argument '%s', using default %.1f°",
                         argv[1], target_angle);
        }
    }

    // Create node with custom parameters
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"theta_ref_deg", target_angle},
                                 {"use_sim_time", true},
                                 {"rate_hz", 250.0},
                                 {"K", std::vector<double>{15.0, 8.0, 2.0}}, // Tuned gains
                                 {"u_min", -785.0},
                                 {"u_max", 785.0},
                                 {"joint_name", "arm_link_joint"},
                                 {"velocity_topic", "/velocity_controller/commands"}});

    try
    {
        g_controller_node = std::make_shared<prop_arm_ctrl::AngleHoldController>(options);

        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "=== MIT PropArm Angle Hold Controller ===");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Target angle: %.1f°", target_angle);
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Reference frame: 0°=horizontal, +90°=up, -90°=down");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Control method: State feedback with velocity commands");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Press Ctrl+C to stop gracefully");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "========================================");

        // Spin the node
        rclcpp::spin(g_controller_node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Controller failed to start: %s", e.what());
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Angle hold controller shut down cleanly");
    rclcpp::shutdown();
    return 0;
}

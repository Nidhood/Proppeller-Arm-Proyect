#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QStandardPaths>
#include <QMessageBox>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "prop_arm_gui/main_window.hpp"
#include "prop_arm_gui/prop_arm_gui_node.hpp"

int main(int argc, char **argv)
{
    try
    {
        // Initialize ROS2 first
        rclcpp::init(argc, argv);

        // Initialize Qt Application
        QApplication app(argc, argv);

        // Application properties
        app.setApplicationName("PropArm Control System");
        app.setApplicationVersion("1.0.0");
        app.setOrganizationName("Aerospace Robotics Lab");
        app.setApplicationDisplayName("PropArm Aerospace GUI");

        // Remove deprecated high DPI attributes for Qt6
        // These are now enabled by default and don't need to be set

        // Set dark fusion style for modern aerospace look
        app.setStyle(QStyleFactory::create("Fusion"));

        // Create dark aerospace palette
        QPalette darkPalette;
        darkPalette.setColor(QPalette::Window, QColor(15, 23, 42));
        darkPalette.setColor(QPalette::WindowText, QColor(241, 245, 249));
        darkPalette.setColor(QPalette::Base, QColor(30, 41, 59));
        darkPalette.setColor(QPalette::AlternateBase, QColor(51, 65, 85));
        darkPalette.setColor(QPalette::ToolTipBase, QColor(241, 245, 249));
        darkPalette.setColor(QPalette::ToolTipText, QColor(15, 23, 42));
        darkPalette.setColor(QPalette::Text, QColor(241, 245, 249));
        darkPalette.setColor(QPalette::Button, QColor(51, 65, 85));
        darkPalette.setColor(QPalette::ButtonText, QColor(241, 245, 249));
        darkPalette.setColor(QPalette::BrightText, QColor(239, 68, 68));
        darkPalette.setColor(QPalette::Link, QColor(59, 130, 246));
        darkPalette.setColor(QPalette::Highlight, QColor(6, 182, 212));
        darkPalette.setColor(QPalette::HighlightedText, QColor(15, 23, 42));
        app.setPalette(darkPalette);

        // Create ROS2 node with error checking
        auto node_options = rclcpp::NodeOptions();
        auto ros_node = std::make_shared<PropArmGuiNode>(node_options);

        if (!ros_node)
        {
            QMessageBox::critical(nullptr, "Error", "Failed to create ROS2 node");
            return 1;
        }

        // Create main window
        MainWindow window(ros_node);
        window.show();

        // Setup graceful shutdown
        QObject::connect(&app, &QApplication::aboutToQuit, [&]()
                         {
            RCLCPP_INFO(ros_node->get_logger(), "Shutting down PropArm GUI...");
            rclcpp::shutdown(); });

        // Log startup information
        RCLCPP_INFO(ros_node->get_logger(),
                    "PropArm Aerospace GUI started successfully");
        RCLCPP_INFO(ros_node->get_logger(),
                    "Monitoring topics: %s, %s",
                    "/prop_arm/arm_angle_deg", "/prop_arm/motor_speed_est");
        RCLCPP_INFO(ros_node->get_logger(),
                    "Publishing to: %s, %s",
                    "/velocity_controller/commands", "/motor_force_controller/commands");

        // Run the application
        int result = app.exec();

        // Cleanup
        RCLCPP_INFO(ros_node->get_logger(), "GUI closed, cleaning up...");

        // Ensure ROS is properly shut down
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        return result;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());

        // Show error dialog if Qt is available
        if (QApplication::instance())
        {
            QMessageBox::critical(nullptr, "Fatal Error",
                                  QString("Exception occurred: %1").arg(e.what()));
        }

        return 1;
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown exception in main");

        // Show error dialog if Qt is available
        if (QApplication::instance())
        {
            QMessageBox::critical(nullptr, "Fatal Error",
                                  "Unknown exception occurred");
        }

        return 1;
    }
}
#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QStandardPaths>
#include <QMessageBox>
#include <QThread>
#include <QSurfaceFormat>
#include <QScreen>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <memory>
#include <thread>

#include "prop_arm_gui/main_window.hpp"
#include "prop_arm_gui/prop_arm_gui_node.hpp"

// Global executor for ROS2
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> g_executor;
std::shared_ptr<PropArmGuiNode> g_ros_node;
std::thread g_ros_thread;
std::atomic<bool> g_shutdown{false};

void rosThreadFunc()
{
    while (rclcpp::ok() && !g_shutdown.load())
    {
        try
        {
            g_executor->spin_some(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Increased sleep time
        }
        catch (const std::exception &e)
        {
            if (g_ros_node)
            {
                RCLCPP_ERROR(g_ros_node->get_logger(), "ROS thread exception: %s", e.what());
            }
            break;
        }
    }
}

int main(int argc, char **argv)
{
    try
    {
        // Initialize ROS2 first
        rclcpp::init(argc, argv);

        // Configure OpenGL surface format for Qt6 compatibility
        QSurfaceFormat format;
        format.setDepthBufferSize(24);
        format.setStencilBufferSize(8);
        format.setVersion(3, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
        format.setSamples(4); // Anti-aliasing
        format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
        format.setRenderableType(QSurfaceFormat::OpenGL);
        QSurfaceFormat::setDefaultFormat(format);

        // Initialize Qt Application with Ubuntu-specific settings
        QApplication app(argc, argv);

        // Ubuntu-specific environment variables
        qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");
        qputenv("QT_SCALE_FACTOR", "1.0");

        // Application properties
        app.setApplicationName("PropArm Control System");
        app.setApplicationVersion("1.0.0");
        app.setOrganizationName("Aerospace Robotics Lab");
        app.setApplicationDisplayName("PropArm Aerospace GUI");

        // Set dark fusion style for modern aerospace look
        app.setStyle(QStyleFactory::create("Fusion"));

        // Create dark aerospace palette optimized for Ubuntu
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
        g_ros_node = std::make_shared<PropArmGuiNode>(node_options);

        if (!g_ros_node)
        {
            QMessageBox::critical(nullptr, "Error", "Failed to create ROS2 node");
            return 1;
        }

        // Create executor and add node
        g_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        g_executor->add_node(g_ros_node);

        // Start ROS2 in separate thread with better error handling
        g_ros_thread = std::thread(rosThreadFunc);

        // Create main window
        MainWindow window(g_ros_node);

        // Ubuntu-specific window management
        window.show();
        window.raise();
        window.activateWindow();

// Force window to front on Ubuntu/X11
#ifdef Q_OS_LINUX
        window.setWindowState(window.windowState() | Qt::WindowActive);

        // Additional Ubuntu compatibility
        QTimer::singleShot(100, [&window]()
                           {
            window.raise();
            window.activateWindow(); });
#endif

        // Setup graceful shutdown with better cleanup
        QObject::connect(&app, &QApplication::aboutToQuit, [&]()
                         {
            if (g_ros_node) {
                RCLCPP_INFO(g_ros_node->get_logger(), "Shutting down PropArm GUI...");
            }
            
            // Signal shutdown
            g_shutdown.store(true);
            
            // Wait for ROS thread with timeout
            if (g_ros_thread.joinable()) {
                auto future = std::async(std::launch::async, [&]() {
                    g_ros_thread.join();
                });
                
                if (future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
                    if (g_ros_node) {
                        RCLCPP_WARN(g_ros_node->get_logger(), "ROS thread join timeout, forcing shutdown");
                    }
                    g_ros_thread.detach();
                }
            }
            
            // Remove node from executor
            if (g_executor && g_ros_node) {
                try {
                    g_executor->remove_node(g_ros_node);
                } catch (const std::exception& e) {
                    // Ignore cleanup errors
                }
            }
            
            // Shutdown ROS
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            } });

        // Log startup information
        if (g_ros_node)
        {
            RCLCPP_INFO(g_ros_node->get_logger(),
                        "PropArm Aerospace GUI started successfully");
            RCLCPP_INFO(g_ros_node->get_logger(),
                        "Monitoring topics: %s, %s",
                        "/prop_arm/arm_angle_deg", "/prop_arm/motor_speed_est");
            RCLCPP_INFO(g_ros_node->get_logger(),
                        "Publishing to: %s, %s",
                        "/velocity_controller/commands", "/motor_force_controller/commands");
        }

        // Process pending events before starting main loop
        app.processEvents();

        // Run the application
        int result = app.exec();

        // Cleanup
        if (g_ros_node)
        {
            RCLCPP_INFO(g_ros_node->get_logger(), "GUI closed, cleaning up...");
        }

        // Signal shutdown and wait for ROS thread
        g_shutdown.store(true);
        if (g_ros_thread.joinable())
        {
            auto future = std::async(std::launch::async, [&]()
                                     { g_ros_thread.join(); });

            if (future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
            {
                g_ros_thread.detach();
            }
        }

        // Cleanup executor
        if (g_executor && g_ros_node)
        {
            try
            {
                g_executor->remove_node(g_ros_node);
            }
            catch (const std::exception &e)
            {
                // Ignore cleanup errors
            }
        }

        // Final ROS shutdown
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        return result;
    }
    catch (const std::exception &e)
    {
        if (g_ros_node)
        {
            RCLCPP_ERROR(g_ros_node->get_logger(), "Exception in main: %s", e.what());
        }

        // Show error dialog if Qt is available
        if (QApplication::instance())
        {
            QMessageBox::critical(nullptr, "Fatal Error",
                                  QString("Exception occurred: %1").arg(e.what()));
        }

        // Cleanup on exception
        g_shutdown.store(true);
        if (g_ros_thread.joinable())
        {
            g_ros_thread.detach(); // Don't wait on exception
        }

        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        return 1;
    }
    catch (...)
    {
        if (g_ros_node)
        {
            RCLCPP_ERROR(g_ros_node->get_logger(), "Unknown exception in main");
        }

        // Show error dialog if Qt is available
        if (QApplication::instance())
        {
            QMessageBox::critical(nullptr, "Fatal Error",
                                  "Unknown exception occurred");
        }

        // Cleanup on exception
        g_shutdown.store(true);
        if (g_ros_thread.joinable())
        {
            g_ros_thread.detach();
        }

        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        return 1;
    }
}
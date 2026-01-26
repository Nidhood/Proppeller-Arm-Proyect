#include "prop_arm_gui/app/ros_qt_bridge.hpp"

#include <QCoreApplication>
#include <QMetaObject>

namespace prop_arm::app {

RosQtBridge::RosQtBridge(QApplication& app, rclcpp::Node::SharedPtr node)
    : app_(app),
      node_(std::move(node)) {
    executor_.add_node(node_);
    spinning_.store(true);
    QObject::connect(&app_, &QCoreApplication::aboutToQuit, [&]() {
        stop();
    });
    ros_thread_ = std::thread([this]() {
        spinLoop_();
    });
    sigint_watcher_ = std::thread([this]() {
        sigintWatchLoop_();
    });
}

RosQtBridge::~RosQtBridge() {
    stopAndJoin_();
}

void RosQtBridge::stop() noexcept {
    spinning_.store(false);
    executor_.cancel();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void RosQtBridge::stopAndJoin_() noexcept {
    stop();

    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
    if (sigint_watcher_.joinable()) {
        sigint_watcher_.join();
    }
}

void RosQtBridge::spinLoop_() {
    using namespace std::chrono_literals;

    while (spinning_.load() && rclcpp::ok()) {
        executor_.spin_some();
        std::this_thread::sleep_for(5ms);
    }
}

void RosQtBridge::sigintWatchLoop_() {
    using namespace std::chrono_literals;

    while (spinning_.load() && rclcpp::ok()) {
        std::this_thread::sleep_for(50ms);
    }

    QMetaObject::invokeMethod(
        &app_,
        "quit",
        Qt::QueuedConnection
    );
}

}  // namespace prop_arm::app

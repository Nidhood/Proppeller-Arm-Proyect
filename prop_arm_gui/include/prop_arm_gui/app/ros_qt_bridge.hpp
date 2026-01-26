#pragma once

#include <QApplication>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <atomic>
#include <thread>

namespace prop_arm::app {

class RosQtBridge final {
public:
    RosQtBridge(QApplication& app, rclcpp::Node::SharedPtr node);
    ~RosQtBridge();
    RosQtBridge(const RosQtBridge&) = delete;
    RosQtBridge& operator=(const RosQtBridge&) = delete;
    void stop() noexcept;

private:
    void spinLoop_();
    void sigintWatchLoop_();
    void stopAndJoin_() noexcept;

private:
    QApplication& app_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::atomic<bool> spinning_{false};
    std::thread ros_thread_;
    std::thread sigint_watcher_;
};

}  // namespace prop_arm::app

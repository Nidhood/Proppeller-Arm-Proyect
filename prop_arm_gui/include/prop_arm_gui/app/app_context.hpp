#pragma once

#include <memory>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

#include "prop_arm_gui/app/main_window.hpp"
#include "prop_arm_gui/core/models/telemetry_store.hpp"
#include "prop_arm_gui/core/services/recorder_service.hpp"
#include "prop_arm_gui/core/services/calibration_service.hpp"
#include "prop_arm_gui/core/services/urdf_updater.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/infra/ros/prop_arm_ros_bridge.hpp"
#include "prop_arm_gui/infra/ros/topic_registry.hpp"
#include "prop_arm_gui/infra/ros/prop_arm_ros_bridge_command_sink.hpp"

namespace prop_arm::app {

class AppContext {
public:
    AppContext();
    ~AppContext();

    MainWindow& mainWindow() noexcept;
    rclcpp::Node::SharedPtr node() const noexcept;

private:
    static std::filesystem::path resolveWorkspacePath_();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<prop_arm::infra::ros::TopicRegistry> topic_registry_;
    std::unique_ptr<prop_arm::infra::ros::LimitsRegistry> limits_registry_;

    prop_arm::models::TelemetryStore telemetry_;
    prop_arm::core::services::RecorderService recorder_;

    MainWindow main_window_;
    std::unique_ptr<prop_arm::infra::ros::PropArmRosBridge> ros_bridge_;

    std::filesystem::path workspace_path_;
    std::unique_ptr<prop_arm::services::UrdfUpdater> urdf_updater_;
    std::unique_ptr<prop_arm::infra::ros::PropArmRosBridgeCommandSink> calib_sink_;
    std::unique_ptr<prop_arm::services::CalibrationService> calibration_service_;
};

} // namespace prop_arm::app

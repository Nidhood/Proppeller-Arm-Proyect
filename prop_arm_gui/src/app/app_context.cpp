#include "prop_arm_gui/app/app_context.hpp"

#include <QObject>
#include <cstdlib>     // std::getenv
#include <stdexcept>

namespace prop_arm::app {

static std::filesystem::path envPathOrEmpty(const char* key) {
    const char* v = std::getenv(key);
    if (!v || !*v) return {};
    return std::filesystem::path(v);
}

std::filesystem::path AppContext::resolveWorkspacePath_() {
    // Priority:
    // 1) PROP_ARM_WS
    // 2) COLCON_PREFIX_PATH (best-effort fallback)
    // 3) ~/prop_arm_ws
    auto ws = envPathOrEmpty("PROP_ARM_WS");
    if (!ws.empty()) return ws;

    // If you want: parse COLCON_PREFIX_PATH; skipping heavy parsing here.
    const auto home = envPathOrEmpty("HOME");
    if (!home.empty()) return home / "prop_arm_ws";

    return std::filesystem::current_path();
}

AppContext::AppContext()
    : node_(std::make_shared<rclcpp::Node>("prop_arm_gui_node")),
      topic_registry_(std::make_shared<prop_arm::infra::ros::TopicRegistry>(*node_)),
      limits_registry_(std::make_unique<prop_arm::infra::ros::LimitsRegistry>(*node_)),
      telemetry_(5000),
      recorder_(),
      main_window_(nullptr),
      workspace_path_(resolveWorkspacePath_())
{
    // 1) ROS bridge
    ros_bridge_ = std::make_unique<prop_arm::infra::ros::PropArmRosBridge>(
                      node_,
                      topic_registry_,
                      &main_window_
                  );

    // 2) MainWindow dependencies
    main_window_.setTelemetryStore(&telemetry_);
    main_window_.setLimitsRegistry(limits_registry_.get());
    main_window_.setRecorderService(&recorder_);

    // 3) Calibration core (IMPORTANT: default ctor, then inject dependencies)
    urdf_updater_ = std::make_unique<prop_arm::services::UrdfUpdater>(workspace_path_);
    calib_sink_ = std::make_unique<prop_arm::infra::ros::PropArmRosBridgeCommandSink>(ros_bridge_.get());

    calibration_service_ = std::make_unique<prop_arm::services::CalibrationService>(&main_window_);
    calibration_service_->setTelemetryStore(&telemetry_);
    calibration_service_->setCommandSink(calib_sink_.get());
    calibration_service_->setUrdfUpdater(urdf_updater_.get());
    // Optional: sim reload hook can be injected later (no-op by default)

    // 4) Give service to calibration UI
    main_window_.setCalibrationService(calibration_service_.get());

    // 5) Bridge -> Store + Recorder + UI
    QObject::connect(
        ros_bridge_.get(),
        &prop_arm::infra::ros::PropArmRosBridge::telemetryUpdated,
        &main_window_,
    [this](prop_arm::models::TelemetrySample sample) {
        telemetry_.push(sample);
        recorder_.onSample(sample);
        main_window_.onTelemetry(sample);
    },
    Qt::QueuedConnection
    );

    // 6) Connection status -> UI
    QObject::connect(
        ros_bridge_.get(),
        &prop_arm::infra::ros::PropArmRosBridge::connectionChanged,
        &main_window_,
        &prop_arm::app::MainWindow::setConnected,
        Qt::QueuedConnection
    );

    // 7) Stream live -> UI (prevents your unused-parameter warning AND fixes overlay logic)
    QObject::connect(
        ros_bridge_.get(),
        &prop_arm::infra::ros::PropArmRosBridge::streamLiveChanged,
        &main_window_,
        &prop_arm::app::MainWindow::setStreamLive,
        Qt::QueuedConnection
    );

    // 8) UI commands -> Bridge publishers (broadcast commands)
    QObject::connect(
        &main_window_,
        &prop_arm::app::MainWindow::refAngleChanged,
        ros_bridge_.get(),
        &prop_arm::infra::ros::PropArmRosBridge::sendRefAngle,
        Qt::QueuedConnection
    );

    QObject::connect(
        &main_window_,
        &prop_arm::app::MainWindow::pwmChanged,
        ros_bridge_.get(),
        &prop_arm::infra::ros::PropArmRosBridge::sendPwm,
        Qt::QueuedConnection
    );

    QObject::connect(
        &main_window_,
        &prop_arm::app::MainWindow::autoModeChanged,
        ros_bridge_.get(),
        &prop_arm::infra::ros::PropArmRosBridge::sendAutoMode,
        Qt::QueuedConnection
    );

    QObject::connect(
        &main_window_,
        &prop_arm::app::MainWindow::stopRequested,
        ros_bridge_.get(),
        &prop_arm::infra::ros::PropArmRosBridge::sendStop,
        Qt::QueuedConnection
    );
}

AppContext::~AppContext() {
    // Stop worker threads safely before dependencies die.
    if (calibration_service_) {
        calibration_service_->stopCalibration();
    }

    ros_bridge_.reset();
    calibration_service_.reset();
    calib_sink_.reset();
    urdf_updater_.reset();

    limits_registry_.reset();
    topic_registry_.reset();
    node_.reset();
}

MainWindow& AppContext::mainWindow() noexcept {
    return main_window_;
}

rclcpp::Node::SharedPtr AppContext::node() const noexcept {
    return node_;
}

} // namespace prop_arm::app

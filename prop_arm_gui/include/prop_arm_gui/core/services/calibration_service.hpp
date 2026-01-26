#pragma once

#include "prop_arm_gui/core/models/calibration_types.hpp"
#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"

#include <QObject>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace prop_arm::models {
class TelemetryStore;
}

namespace prop_arm::services {

class UrdfUpdater;

// Domain-specific command sink used for calibration experiments.
class ICalibrationCommandSink {
public:
    virtual ~ICalibrationCommandSink() = default;

    virtual void sendPwmUs(prop_arm::models::Domain domain, std::uint16_t pwm_us) = 0;
    virtual void sendRefAngleRad(prop_arm::models::Domain domain, double ref_angle_rad) = 0;
    virtual void sendAutoMode(prop_arm::models::Domain domain, bool enabled) = 0;
    virtual void sendStop(prop_arm::models::Domain domain) = 0;
};

// Optional: implement a sim reloader (respawn/reload entity).
class ISimReloadHook {
public:
    virtual ~ISimReloadHook() = default;
    virtual bool reloadSimulation() = 0; // return false if not supported
};

class CalibrationService final : public QObject {
    Q_OBJECT

public:
    // Default ctor (wire using setters)
    explicit CalibrationService(QObject* parent = nullptr);

    // Convenience ctor (wire in one shot) - FIXES your make_unique(...) error
    explicit CalibrationService(prop_arm::models::TelemetryStore* store,
                                ICalibrationCommandSink* sink,
                                prop_arm::services::UrdfUpdater* urdf_updater,
                                ISimReloadHook* sim_reload = nullptr,
                                QObject* parent = nullptr);

    ~CalibrationService() override;

    CalibrationService(const CalibrationService&) = delete;
    CalibrationService& operator=(const CalibrationService&) = delete;

    // Wiring (safe to call any time; typically before starting)
    void setTelemetryStore(prop_arm::models::TelemetryStore* store) noexcept;
    void setCommandSink(ICalibrationCommandSink* sink) noexcept;
    void setUrdfUpdater(prop_arm::services::UrdfUpdater* updater) noexcept;
    void setSimReloadHook(ISimReloadHook* hook) noexcept;

    // API used by CalibrationPage
    void startCalibration(const prop_arm::models::CalibrationConfig& cfg);
    void stopCalibration();
    void applyCalibration();        // writes to xacro via UrdfUpdater
    void resetToDefaults();

    [[nodiscard]] prop_arm::models::CalibrationResult getLastResult() const;
    [[nodiscard]] prop_arm::models::MotorVelocityParams getMotorParams() const;
    [[nodiscard]] prop_arm::models::ArmPhysicsParams getPhysicsParams() const;

signals:
    void statusChanged(prop_arm::models::CalibrationStatus status);
    void progressUpdated(double progress01);
    void calibrationCompleted(prop_arm::models::CalibrationResult result);
    void metricsUpdated(prop_arm::models::CalibrationMetrics metrics);
    void parametersChanged();

private:
    void runWorker_(prop_arm::models::CalibrationConfig cfg);

    // One evaluation = apply params (sim) + run experiment + compute cost
    double evaluateMotorCost_(const prop_arm::models::CalibrationConfig& cfg,
                              const prop_arm::models::MotorVelocityParams& p,
                              prop_arm::models::CalibrationMetrics* out_metrics);

    double evaluatePhysicsCost_(const prop_arm::models::CalibrationConfig& cfg,
                                const prop_arm::models::ArmPhysicsParams& p,
                                prop_arm::models::CalibrationMetrics* out_metrics);

    // Experiment execution
    bool runMotorExperiment_(const prop_arm::models::CalibrationConfig& cfg);
    bool runPhysicsExperiment_(const prop_arm::models::CalibrationConfig& cfg);

    // Data window extraction (by steady_clock time window)
    std::vector<prop_arm::models::TelemetrySample> collectWindow_(
        prop_arm::models::Domain domain,
        std::chrono::steady_clock::time_point t0,
        std::chrono::steady_clock::time_point t1,
        std::size_t max_points) const;

    // Helpers
    static prop_arm::models::CalibrationMetrics makeMetrics_(
        double rmse, double max_error, double mean_error, double corr, std::size_t n);

    void setStatus_(prop_arm::models::CalibrationStatus s);
    void setProgress_(double p01);

    [[nodiscard]] bool canRun_() const noexcept;

private:
    prop_arm::models::TelemetryStore* store_{nullptr};
    ICalibrationCommandSink* sink_{nullptr};
    prop_arm::services::UrdfUpdater* urdf_updater_{nullptr};
    ISimReloadHook* sim_reload_{nullptr};

    std::atomic_bool stop_{false};
    std::atomic_bool running_{false};

    mutable std::mutex state_mtx_;
    prop_arm::models::MotorVelocityParams motor_params_{};
    prop_arm::models::ArmPhysicsParams physics_params_{};
    prop_arm::models::CalibrationResult last_result_{};
    prop_arm::models::CalibrationStatus status_{prop_arm::models::CalibrationStatus::Idle};

    std::thread worker_;
};

}  // namespace prop_arm::services

#pragma once

#include "prop_arm_gui/core/models/telemetry_store.hpp"
#include "prop_arm_gui/core/services/calibration_interfaces.hpp"
#include "prop_arm_gui/infra/ros/calibration_command_sink.hpp"

#include <chrono>
#include <memory>
#include <stop_token>
#include <vector>

namespace prop_arm::infra::ros {

class StepCalibrationCommander final : public prop_arm::services::ICalibrationCommander {
public:
    explicit StepCalibrationCommander(
        std::shared_ptr<prop_arm::models::TelemetryStore> store,
        std::shared_ptr<ICalibrationCommandSink> sink);

    bool runMotorSequence(
        const std::vector<double>& pwm_steps,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st) override;

    bool runPhysicsSequence(
        const std::vector<double>& ref_steps_rad,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st) override;

private:
    bool sleepOrStop_(std::chrono::duration<double> d, std::stop_token st) const;

    void clearTelemetry_() const;
    void broadcastPwm_(std::uint16_t pwm_us) const;
    void broadcastRefAngle_(double rad) const;

private:
    std::shared_ptr<prop_arm::models::TelemetryStore> store_;
    std::shared_ptr<ICalibrationCommandSink> sink_;
};

}  // namespace prop_arm::infra::ros

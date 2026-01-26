#include "prop_arm_gui/infra/ros/step_calibration_commander.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace prop_arm::infra::ros {

namespace {

std::uint16_t clampPwm(double v) {
    const auto x = static_cast<int>(std::lround(v));
    return static_cast<std::uint16_t>(std::clamp(x, 800, 2200));
}

}  // namespace

StepCalibrationCommander::StepCalibrationCommander(
    std::shared_ptr<prop_arm::models::TelemetryStore> store,
    std::shared_ptr<ICalibrationCommandSink> sink)
    : store_(std::move(store)),
      sink_(std::move(sink)) {}

bool StepCalibrationCommander::runMotorSequence(
    const std::vector<double>& pwm_steps,
    double step_duration_sec,
    double settling_time_sec,
    std::stop_token st)
{
    if (!sink_ || !store_) return false;
    if (pwm_steps.empty()) return false;

    clearTelemetry_();

    // Settling before first step (optional, but helps to start from quiet state)
    if (!sleepOrStop_(std::chrono::duration<double>(std::max(0.0, settling_time_sec)), st)) return false;

    for (double pwm : pwm_steps) {
        if (st.stop_requested()) return false;

        broadcastPwm_(clampPwm(pwm));

        // Let the system settle before collecting meaningful data at this step
        if (!sleepOrStop_(std::chrono::duration<double>(std::max(0.0, settling_time_sec)), st)) return false;

        // Hold this input to accumulate samples
        if (!sleepOrStop_(std::chrono::duration<double>(std::max(0.0, step_duration_sec)), st)) return false;
    }

    return !st.stop_requested();
}

bool StepCalibrationCommander::runPhysicsSequence(
    const std::vector<double>& ref_steps_rad,
    double step_duration_sec,
    double settling_time_sec,
    std::stop_token st)
{
    if (!sink_ || !store_) return false;
    if (ref_steps_rad.empty()) return false;

    clearTelemetry_();

    if (!sleepOrStop_(std::chrono::duration<double>(std::max(0.0, settling_time_sec)), st)) return false;

    for (double rad : ref_steps_rad) {
        if (st.stop_requested()) return false;

        broadcastRefAngle_(rad);

        if (!sleepOrStop_(std::chrono::duration<double>(std::max(0.0, settling_time_sec)), st)) return false;
        if (!sleepOrStop_(std::chrono::duration<double>(std::max(0.0, step_duration_sec)), st)) return false;
    }

    return !st.stop_requested();
}

bool StepCalibrationCommander::sleepOrStop_(std::chrono::duration<double> d, std::stop_token st) const {
    // Granular sleep to be responsive to stop requests.
    constexpr auto kTick = std::chrono::milliseconds(20);
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(d);

    while (remaining.count() > 0) {
        if (st.stop_requested()) return false;
        const auto step = (remaining > kTick) ? kTick : remaining;
        std::this_thread::sleep_for(step);
        remaining -= step;
    }

    return !st.stop_requested();
}

void StepCalibrationCommander::clearTelemetry_() const {
    store_->clearDomain(prop_arm::models::Domain::Real);
    store_->clearDomain(prop_arm::models::Domain::Sim);
}

void StepCalibrationCommander::broadcastPwm_(std::uint16_t pwm_us) const {
    sink_->sendPwmUs(prop_arm::models::Domain::Real, pwm_us);
    sink_->sendPwmUs(prop_arm::models::Domain::Sim, pwm_us);
}

void StepCalibrationCommander::broadcastRefAngle_(double rad) const {
    sink_->sendRefAngleRad(prop_arm::models::Domain::Real, rad);
    sink_->sendRefAngleRad(prop_arm::models::Domain::Sim, rad);
}

}  // namespace prop_arm::infra::ros

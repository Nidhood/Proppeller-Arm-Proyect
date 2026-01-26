#pragma once

#include "prop_arm_gui/core/models/calibration_types.hpp"

#include <stop_token>
#include <vector>

namespace prop_arm::services {

// Responsible for applying candidate parameters to the simulator (URDF/xacro, ROS params, reload, etc.)
class ISimParamApplier {
public:
    virtual ~ISimParamApplier() = default;

    virtual bool applyMotorParams(const prop_arm::models::MotorVelocityParams& p) = 0;
    virtual bool applyPhysicsParams(const prop_arm::models::ArmPhysicsParams& p) = 0;

    // Optional: if your sim needs a "reload/restart" after modifying URDF
    virtual bool reloadSimulation() = 0;
};

// Responsible for running the *same* excitation on Real and Sim.
// Implementation typically publishes reference commands to both domains (or a shared command topic).
class ICalibrationCommander {
public:
    virtual ~ICalibrationCommander() = default;

    // MotorVelocity calibration: command a PWM sequence and block until done.
    // Must return true if completed without interruption.
    virtual bool runMotorSequence(
        const std::vector<double>& pwm_steps,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st) = 0;

    // ArmPhysics calibration: command an angle reference (or other trajectory) sequence and block until done.
    virtual bool runPhysicsSequence(
        const std::vector<double>& ref_steps_rad,
        double step_duration_sec,
        double settling_time_sec,
        std::stop_token st) = 0;
};

}  // namespace prop_arm::services

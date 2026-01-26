#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace prop_arm::models {

enum class CalibrationType : std::uint8_t {
    MotorVelocity,
    ArmPhysics
};

enum class CalibrationStatus : std::uint8_t {
    Idle,
    Running,
    Analyzing,
    Completed,
    Failed
};

struct MotorVelocityParams {
    double Kw{1.4016};           // Motor gain [rad/s/us]
    double tau_w{0.564};         // Motor time constant [s]
    double L_w{0.294};           // Motor delay [s]
    double motor_cmd_scale{1.162}; // Motor command scale factor

    bool operator==(const MotorVelocityParams&) const noexcept = default;
};

struct ArmPhysicsParams {
    double mass{0.4067};         // Arm mass [kg]
    double inertia_yy{0.01086};  // Moment of inertia around Y axis [kg·m²]
    double damping{0.0326233};   // Joint damping [N·m·s/rad]
    double friction{0.0};        // Joint friction [N·m]

    bool operator==(const ArmPhysicsParams&) const noexcept = default;
};

struct CalibrationMetrics {
    double rmse{0.0};            // Root mean square error
    double max_error{0.0};       // Maximum error
    double mean_error{0.0};      // Mean error
    double correlation{0.0};     // Correlation coefficient
    std::size_t sample_count{0}; // Number of samples used

    bool operator==(const CalibrationMetrics&) const noexcept = default;
};

struct CalibrationResult {
    CalibrationStatus status{CalibrationStatus::Idle};
    CalibrationType type{CalibrationType::MotorVelocity};
    CalibrationMetrics metrics;
    MotorVelocityParams motor_params;
    ArmPhysicsParams physics_params;
    std::string error_message;

    bool operator==(const CalibrationResult&) const noexcept = default;
};

struct CalibrationConfig {
    CalibrationType type{CalibrationType::MotorVelocity};
    double duration_sec{30.0};        // Test duration
    double settling_time_sec{2.0};    // Time to wait before recording
    std::vector<double> test_inputs;  // Test PWM values or angles
    bool auto_apply{false};           // Auto-apply on success
    double convergence_threshold{0.01}; // Convergence criterion
    std::size_t max_iterations{50};   // Max optimization iterations

    bool operator==(const CalibrationConfig&) const noexcept = default;
};

} // namespace prop_arm::models
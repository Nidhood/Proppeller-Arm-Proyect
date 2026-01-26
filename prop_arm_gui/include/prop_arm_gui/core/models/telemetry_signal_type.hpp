#pragma once

namespace prop_arm::models {

enum class TelemetrySignalType {
    Angle,
    MotorSpeed,
    PwmFeedback,
    AngleRef
};

} // namespace prop_arm::models
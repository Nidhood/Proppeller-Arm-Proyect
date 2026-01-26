#pragma once

#include "prop_arm_gui/core/models/range_type.hpp"

namespace prop_arm::models {

struct Limits {
    Range<double> angle_deg;
    Range<double> motor_speed;
    Range<int> pwm_us;
    Range<double> duty_pct{};
    Range<double> error_deg{};
    Range<double> error_tracking_deg{};
};

} // namespace prop_arm::models

#pragma once

#include "prop_arm_gui/core/models/domain.hpp"

#include <chrono>
#include <cstdint>

namespace prop_arm::models {

struct TelemetrySample {
    Domain domain{Domain::Real};
    std::chrono::steady_clock::time_point t{};
    double arm_angle_rad{0.0};
    double motor_speed_rad_s{0.0};
    double ref_angle_rad{0.0};
    std::uint16_t pwm_us{0};

    bool valid{false};

    bool operator==(const TelemetrySample&) const noexcept = default;
};

} // namespace prop_arm::models
#pragma once

#include <numbers>

namespace prop_arm::core::math {

// Angle:
[[nodiscard]] constexpr inline double rad_to_deg(double rad) noexcept {
    return rad * 180.0 / std::numbers::pi;
}
[[nodiscard]] constexpr inline double deg_to_rad(double deg) noexcept {
    return deg * std::numbers::pi / 180.0;
}

// Angular speed:
[[nodiscard]] constexpr inline double rpm_to_rad_s(double rpm) noexcept {
    return rpm * 2.0 * std::numbers::pi / 60.0;
}
[[nodiscard]] constexpr inline double rad_s_to_rpm(double rad_s) noexcept {
    return rad_s * 60.0 / (2.0 * std::numbers::pi);
}

// Linear speed from angular speed: v = w * r:
[[nodiscard]] constexpr inline double rad_s_to_m_s(double rad_s, double radius_m) noexcept {
    return rad_s * radius_m;
}
[[nodiscard]] constexpr inline double m_s_to_rad_s(double v_m_s, double radius_m) noexcept {
    return (radius_m != 0.0) ? (v_m_s / radius_m) : 0.0;
}

} // namespace prop_arm::core::math

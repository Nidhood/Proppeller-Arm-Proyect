#pragma once

#include <rclcpp/rclcpp.hpp>
#include "prop_arm_gui/core/models/limits_type.hpp"
#include "prop_arm_gui/core/models/range_type.hpp"

namespace prop_arm::infra::ros {

class LimitsRegistry {
public:
    explicit LimitsRegistry(rclcpp::Node& node);

    // Typed accessors:
    [[nodiscard]] const models::Range<double>& angleLimits() const noexcept;
    [[nodiscard]] const models::Range<double>& motorSpeedLimits() const noexcept;
    [[nodiscard]] const models::Range<int>& pwmLimits() const noexcept;
    [[nodiscard]] const models::Range<double>& dutyLimits() const noexcept;
    [[nodiscard]] const models::Range<double>& errorLimits() const noexcept;
    [[nodiscard]] const models::Range<double>& errorTrackingLimits() const noexcept;
    [[nodiscard]] const models::Limits& limits() const noexcept;

private:
    models::Limits limits_;
};

} // namespace prop_arm::infra::ros
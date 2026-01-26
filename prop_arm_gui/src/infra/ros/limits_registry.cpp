#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/core/models/limits_type.hpp"
#include "prop_arm_gui/core/models/range_type.hpp"
#include <stdexcept>

namespace prop_arm::infra::ros {

LimitsRegistry::LimitsRegistry(rclcpp::Node& node) {

    // Declare defaults for chart Y-axis limits:
    node.declare_parameter<double>("limits.angle.min", 0.0);
    node.declare_parameter<double>("limits.angle.max", 140.0);

    node.declare_parameter<double>("limits.motor_speed.min", 0.0);
    node.declare_parameter<double>("limits.motor_speed.max", 1600.0);

    node.declare_parameter<int>("limits.pwm.min", 1000);
    node.declare_parameter<int>("limits.pwm.max", 2000);

    node.declare_parameter<double>("limits.duty.min", 0.0);
    node.declare_parameter<double>("limits.duty.max", 100.0);

    node.declare_parameter<double>("limits.error.min", 0.0);
    node.declare_parameter<double>("limits.error.max", 90.0);

    node.declare_parameter<double>("limits.error_tracking.min", 0.0);
    node.declare_parameter<double>("limits.error_tracking.max", 90.0);

    // Read final values from YAML (if exists):
    limits_.angle_deg.min = node.get_parameter("limits.angle.min").as_double();
    limits_.angle_deg.max = node.get_parameter("limits.angle.max").as_double();
    limits_.angle_deg.validate("limits.angle");

    limits_.motor_speed.min = node.get_parameter("limits.motor_speed.min").as_double();
    limits_.motor_speed.max = node.get_parameter("limits.motor_speed.max").as_double();
    limits_.motor_speed.validate("limits.motor_speed");

    limits_.pwm_us.min = node.get_parameter("limits.pwm.min").as_int();
    limits_.pwm_us.max = node.get_parameter("limits.pwm.max").as_int();
    limits_.pwm_us.validate("limits.pwm_us");

    limits_.duty_pct.min = node.get_parameter("limits.duty.min").as_double();
    limits_.duty_pct.max = node.get_parameter("limits.duty.max").as_double();
    limits_.duty_pct.validate("limits.duty");

    limits_.error_deg.min = node.get_parameter("limits.error.min").as_double();
    limits_.error_deg.max = node.get_parameter("limits.error.max").as_double();
    limits_.error_deg.validate("limits.error");

    limits_.error_tracking_deg.min = node.get_parameter("limits.error_tracking.min").as_double();
    limits_.error_tracking_deg.max = node.get_parameter("limits.error_tracking.max").as_double();
    limits_.error_tracking_deg.validate("limits.error_tracking");
}

// Get angle range.
const models::Range<double>& LimitsRegistry::angleLimits() const noexcept {
    return limits_.angle_deg;
}

// Get motor speed range.
const models::Range<double>& LimitsRegistry::motorSpeedLimits() const noexcept {
    return limits_.motor_speed;
}

// Get pwm us range.
const models::Range<int>& LimitsRegistry::pwmLimits() const noexcept {
    return limits_.pwm_us;
}

// Get duty range.
const models::Range<double>& LimitsRegistry::dutyLimits() const noexcept {
    return limits_.duty_pct;
}

// Get error range.
const models::Range<double>& LimitsRegistry::errorLimits() const noexcept {
    return limits_.error_deg;
}

// Get error tracking range.
const models::Range<double>& LimitsRegistry::errorTrackingLimits() const noexcept {
    return limits_.error_tracking_deg;
}

// Get limits.
const models::Limits& LimitsRegistry::limits() const noexcept {
    return limits_;
}

} // namespace prop_arm::infra::ros
#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "prop_arm_gui/core/models/command_type.hpp"
#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_signal_type.hpp"

namespace prop_arm::infra::ros {

class TopicRegistry final {
public:
    explicit TopicRegistry(rclcpp::Node& node);

    [[nodiscard]] std::string topic(prop_arm::models::Domain d,
                                    prop_arm::models::TelemetrySignalType sig) const;

    [[nodiscard]] std::string topic(prop_arm::models::Domain d,
                                    prop_arm::models::CommandType cmd) const;

private:
    static std::string join_(const std::string& ns, const std::string& name);
    static std::string normalizeNs_(std::string ns);

    // namespaces
    std::string real_ns_;
    std::string sim_ns_;

    // signals
    std::string angle_;
    std::string motor_speed_;
    std::string pwm_feedback_;
    std::string angle_ref_;

    // commands
    std::string ref_angle_;
    std::string pwm_cmd_;
    std::string auto_mode_;
};

}  // namespace prop_arm::infra::ros

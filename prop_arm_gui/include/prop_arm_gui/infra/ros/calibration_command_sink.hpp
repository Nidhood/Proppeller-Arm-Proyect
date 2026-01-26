#pragma once

#include "prop_arm_gui/core/models/domain.hpp"

#include <cstdint>

namespace prop_arm::infra::ros {

class ICalibrationCommandSink {
public:
    virtual ~ICalibrationCommandSink() = default;

    virtual void sendPwmUs(prop_arm::models::Domain domain, std::uint16_t pwm_us) = 0;
    virtual void sendRefAngleRad(prop_arm::models::Domain domain, double ref_angle_rad) = 0;
};

}  // namespace prop_arm::infra::ros

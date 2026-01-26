#pragma once

#include "prop_arm_gui/core/services/calibration_service.hpp" // ICalibrationCommandSink
#include "prop_arm_gui/core/models/domain.hpp"

#include <cstdint>

namespace prop_arm::infra::ros {

class PropArmRosBridge;

class PropArmRosBridgeCommandSink final : public prop_arm::services::ICalibrationCommandSink {
public:
    explicit PropArmRosBridgeCommandSink(PropArmRosBridge* bridge) noexcept;

    void sendPwmUs(prop_arm::models::Domain domain, std::uint16_t pwm_us) override;
    void sendRefAngleRad(prop_arm::models::Domain domain, double ref_angle_rad) override;
    void sendAutoMode(prop_arm::models::Domain domain, bool enabled) override;
    void sendStop(prop_arm::models::Domain domain) override;

private:
    PropArmRosBridge* bridge_{nullptr}; // non-owning
};

}  // namespace prop_arm::infra::ros

#include "prop_arm_gui/infra/ros/prop_arm_ros_bridge_command_sink.hpp"

#include "prop_arm_gui/infra/ros/prop_arm_ros_bridge.hpp"

namespace prop_arm::infra::ros {

PropArmRosBridgeCommandSink::PropArmRosBridgeCommandSink(PropArmRosBridge* bridge) noexcept
    : bridge_(bridge) {}

void PropArmRosBridgeCommandSink::sendPwmUs(prop_arm::models::Domain domain, std::uint16_t pwm_us) {
    if (!bridge_) return;
    bridge_->sendPwmCommand(domain, pwm_us);
}

void PropArmRosBridgeCommandSink::sendRefAngleRad(prop_arm::models::Domain domain, double ref_angle_rad) {
    if (!bridge_) return;
    bridge_->sendAngleReference(domain, ref_angle_rad);
}

void PropArmRosBridgeCommandSink::sendAutoMode(prop_arm::models::Domain domain, bool enabled) {
    if (!bridge_) return;
    bridge_->sendAutoModeCommand(domain, enabled);
}

void PropArmRosBridgeCommandSink::sendStop(prop_arm::models::Domain domain) {
    if (!bridge_) return;
    bridge_->sendAutoModeCommand(domain, false);
    bridge_->sendPwmCommand(domain, static_cast<std::uint16_t>(0));
}

}  // namespace prop_arm::infra::ros

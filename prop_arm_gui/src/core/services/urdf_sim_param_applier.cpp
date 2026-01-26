#include "prop_arm_gui/core/services/urdf_sim_param_applier.hpp"

namespace prop_arm::services {

UrdfSimParamApplier::UrdfSimParamApplier(std::shared_ptr<UrdfUpdater> updater,
        ReloadFn reload_fn)
    : updater_(std::move(updater)),
      reload_fn_(std::move(reload_fn)) {}

bool UrdfSimParamApplier::applyMotorParams(const prop_arm::models::MotorVelocityParams& p) {
    if (!updater_) return false;
    return updater_->updateMotorParameters(p);
}

bool UrdfSimParamApplier::applyPhysicsParams(const prop_arm::models::ArmPhysicsParams& p) {
    if (!updater_) return false;
    return updater_->updatePhysicsParameters(p);
}

bool UrdfSimParamApplier::reloadSimulation() {
    if (!reload_fn_) return true;
    return reload_fn_();
}

std::optional<std::string> UrdfSimParamApplier::lastError() const noexcept {
    if (!updater_) return std::nullopt;
    return updater_->getLastError();
}

}  // namespace prop_arm::services

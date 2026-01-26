#pragma once

#include "prop_arm_gui/core/services/calibration_interfaces.hpp"
#include "prop_arm_gui/core/services/urdf_updater.hpp"

#include <functional>
#include <memory>

namespace prop_arm::services {

class UrdfSimParamApplier final : public ISimParamApplier {
public:
    using ReloadFn = std::function<bool()>;
    explicit UrdfSimParamApplier(std::shared_ptr<UrdfUpdater> updater,
                                 ReloadFn reload_fn = {});
    bool applyMotorParams(const prop_arm::models::MotorVelocityParams& p) override;
    bool applyPhysicsParams(const prop_arm::models::ArmPhysicsParams& p) override;
    bool reloadSimulation() override;
    [[nodiscard]] std::optional<std::string> lastError() const noexcept;

private:
    std::shared_ptr<UrdfUpdater> updater_;
    ReloadFn reload_fn_;
};

}  // namespace prop_arm::services

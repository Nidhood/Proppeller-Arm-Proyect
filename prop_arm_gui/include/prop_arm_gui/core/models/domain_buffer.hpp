#pragma once

#include <optional>
#include <vector>

#include "prop_arm_gui/core/models/telemetry_sample.hpp"

namespace prop_arm::models {

struct DomainBuffer {
    std::optional<TelemetrySample> latest;
    std::vector<TelemetrySample> history;
};

} // namespace prop_arm::models
#pragma once

#include "prop_arm_gui/core/models/recording_state.hpp"
#include <cstddef>
namespace prop_arm::models {

struct RecordingStats {
    std::size_t points_total = 0;
    std::size_t points_real = 0;
    std::size_t points_sim = 0;

    double elapsed_s = 0.0;
    double remaining_s = 0.0;
    double duration_s = 0.0;

    models::RecordingState state = models::RecordingState::Idle;
};

} // namespace prop_arm::domain
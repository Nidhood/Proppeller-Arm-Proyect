#pragma once

#include <cstddef>
#include <functional>
#include <mutex>
#include <vector>

#include "prop_arm_gui/core/logging/data_recorder.hpp"
#include "prop_arm_gui/core/models/recording_state.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/recording_stats.hpp"

namespace prop_arm::core::services {

class RecorderService final {
public:
    using OnProgressFn = std::function<void(double remaining_s, std::size_t points_total)>;
    using OnCompletedFn = std::function<void(std::size_t points_total, double duration_s)>;

    RecorderService();

    void start(double duration_s);
    void stop();
    void clear();

    [[nodiscard]] bool isRecording() const;

    // Feed samples (call from AppContext telemetry fan-out)
    void onSample(const prop_arm::models::TelemetrySample& sample);

    [[nodiscard]] std::vector<prop_arm::models::TelemetrySample> recorded() const;
    [[nodiscard]] prop_arm::core::logging::RecordingStats stats() const;

    void setOnProgress(OnProgressFn cb);
    void setOnCompleted(OnCompletedFn cb);

private:
    void maybeEmitCallbacksUnlocked_(const prop_arm::core::logging::RecordingStats& st);

private:
    mutable std::mutex mtx_;
    prop_arm::core::logging::DataRecorder recorder_;

    OnProgressFn on_progress_{};
    OnCompletedFn on_completed_{};

    // Guard / debouncing
    prop_arm::core::logging::RecordingState last_state_{prop_arm::core::logging::RecordingState::Idle};
    double last_progress_remaining_s_{-1.0};
    bool completed_emitted_{false};
};

}  // namespace prop_arm::core::services

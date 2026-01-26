#pragma once

#include <vector>
#include <mutex>

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/recording_stats.hpp"


namespace prop_arm::core::logging {

using namespace prop_arm::models;
using SteadyClock = std::chrono::steady_clock;

class DataRecorder {
public:
    DataRecorder() = default;
    void start(double duration_s);
    void stop();
    void clear();
    bool isRecording() const;
    RecordingState state() const;
    void pushSample(const TelemetrySample& sample);
    std::vector<TelemetrySample> recorded() const;
    std::vector<TelemetrySample> recorded(Domain d) const;
    RecordingStats stats() const;

private:
    bool withinWindow_(const SteadyClock::time_point& t) const;
    static double seconds_(const SteadyClock::duration& d);
    mutable std::mutex mtx_;
    RecordingState state_ = RecordingState::Idle;
    double duration_s_ = 0.0;
    SteadyClock::time_point t0_{};
    SteadyClock::time_point t_end_{};
    std::vector<TelemetrySample> data_;
};

} // namespace prop_arm::core::logging
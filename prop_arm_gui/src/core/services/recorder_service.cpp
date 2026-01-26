#include "prop_arm_gui/core/services/recorder_service.hpp"

namespace prop_arm::core::services {

RecorderService::RecorderService() = default;

void RecorderService::start(double duration_s) {
    std::lock_guard<std::mutex> lock(mtx_);
    recorder_.start(duration_s);
    last_state_ = prop_arm::core::logging::RecordingState::Recording;
    last_progress_remaining_s_ = -1.0;
    completed_emitted_ = false;
}

void RecorderService::stop() {
    std::lock_guard<std::mutex> lock(mtx_);
    recorder_.stop();
}

void RecorderService::clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    recorder_.clear();
    last_state_ = prop_arm::core::logging::RecordingState::Idle;
    last_progress_remaining_s_ = -1.0;
    completed_emitted_ = false;
}

bool RecorderService::isRecording() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return recorder_.isRecording();
}

void RecorderService::onSample(const prop_arm::models::TelemetrySample& sample) {
    OnProgressFn progress_cb;
    OnCompletedFn completed_cb;
    prop_arm::core::logging::RecordingStats st;

    // Snapshot under lock + update guards that must be consistent
    {
        std::lock_guard<std::mutex> lock(mtx_);
        recorder_.pushSample(sample);
        st = recorder_.stats();

        progress_cb = on_progress_;
        completed_cb = on_completed_;

        if ((progress_cb || completed_cb) && st.state != last_state_) {
            last_state_ = st.state;
            if (st.state == prop_arm::core::logging::RecordingState::Recording) {
                completed_emitted_ = false;
                last_progress_remaining_s_ = -1.0;
            }
        }
    }  // unlock

    if (!progress_cb && !completed_cb) return;

    // Progress: emit when remaining decreases by >= 50ms (or first time)
    if (progress_cb && st.state == prop_arm::core::logging::RecordingState::Recording) {
        bool should_emit = false;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (last_progress_remaining_s_ < 0.0) {
                should_emit = true;
            } else if (st.remaining_s <= (last_progress_remaining_s_ - 0.05)) {
                should_emit = true;
            }

            if (should_emit) {
                last_progress_remaining_s_ = st.remaining_s;
            }
        }

        if (should_emit) {
            progress_cb(st.remaining_s, st.points_total);
        }
    }

    // Completed: emit ONCE when state becomes Completed
    if (completed_cb && st.state == prop_arm::core::logging::RecordingState::Completed) {
        bool should_emit = false;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!completed_emitted_) {
                completed_emitted_ = true;
                should_emit = true;
            }
        }

        if (should_emit) {
            completed_cb(st.points_total, st.duration_s);
        }
    }
}

std::vector<prop_arm::models::TelemetrySample> RecorderService::recorded() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return recorder_.recorded();
}

prop_arm::core::logging::RecordingStats RecorderService::stats() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return recorder_.stats();
}

void RecorderService::setOnProgress(OnProgressFn cb) {
    std::lock_guard<std::mutex> lock(mtx_);
    on_progress_ = std::move(cb);
}

void RecorderService::setOnCompleted(OnCompletedFn cb) {
    std::lock_guard<std::mutex> lock(mtx_);
    on_completed_ = std::move(cb);
}

}  // namespace prop_arm::core::services

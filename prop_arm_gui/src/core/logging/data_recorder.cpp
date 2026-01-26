#include "prop_arm_gui/core/logging/data_recorder.hpp"
#include "prop_arm_gui/core/models/recording_stats.hpp"

namespace prop_arm::core::logging {

void DataRecorder::start(double duration_s) {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.clear();
    duration_s_ = std::max(0.0, duration_s);
    t0_ = SteadyClock::now();
    t_end_ = t0_ + std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(duration_s_));
    state_ = RecordingState::Recording;
}

void DataRecorder::stop() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (state_ == RecordingState::Recording) {
        state_ = RecordingState::Stopped;
    }
}


void DataRecorder::clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.clear();
    state_ = RecordingState::Idle;
    duration_s_ = 0.0;
}

bool DataRecorder::isRecording() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return state_ == RecordingState::Recording;
}

RecordingState DataRecorder::state() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return state_;
}

void DataRecorder::pushSample(const prop_arm::models::TelemetrySample& sample) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (state_ != RecordingState::Recording) return;
    if (!sample.valid) return;
    const auto now = SteadyClock::now();
    if (now >= t_end_) {
        state_ = RecordingState::Completed;
        return;
    }

    data_.push_back(sample);
}


std::vector<prop_arm::models::TelemetrySample> DataRecorder::recorded() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

std::vector<prop_arm::models::TelemetrySample>
DataRecorder::recorded(prop_arm::models::Domain domain) const {
    std::lock_guard<std::mutex> lock(mtx_);

    std::vector<prop_arm::models::TelemetrySample> out;
    out.reserve(data_.size());

    for (const auto& s : data_) {
        if (s.domain == domain) out.push_back(s);
    }

    return out;
}

RecordingStats DataRecorder::stats() const {
    std::lock_guard<std::mutex> lock(mtx_);

    RecordingStats st;
    st.state = state_;
    st.duration_s = duration_s_;
    st.points_total = data_.size();

    for (const auto& s : data_) {
        if (s.domain == prop_arm::models::Domain::Real) st.points_real++;
        else st.points_sim++;
    }

    if (state_ == RecordingState::Recording) {
        const auto now = SteadyClock::now();
        const auto elapsed = now - t0_;
        st.elapsed_s = seconds_(elapsed);

        const auto remaining = (t_end_ > now) ? (t_end_ - now) : SteadyClock::duration::zero();
        st.remaining_s = std::max(0.0, seconds_(remaining));
    }

    return st;
}

bool DataRecorder::withinWindow_(const SteadyClock::time_point& t) const {
    return (t >= t0_) && (t < t_end_);
}

double DataRecorder::seconds_(const SteadyClock::duration& d) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(d).count();
}

} // namespace prop_arm::core::logging
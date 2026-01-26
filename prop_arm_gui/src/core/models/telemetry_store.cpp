#include "prop_arm_gui/core/models/telemetry_store.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace prop_arm::models {

TelemetryStore::TelemetryStore(std::size_t max_history)
    : max_history_(std::max<std::size_t>(1, max_history))
{
    buffers_.emplace(Domain::Real, DomainBuffer{});
    buffers_.emplace(Domain::Sim, DomainBuffer{});
}

void TelemetryStore::setMaxHistory(std::size_t max_history)
{
    std::lock_guard<std::mutex> lock(mtx_);
    max_history_ = std::max<std::size_t>(1, max_history);

    for (auto& [_, buf] : buffers_) {
        if (buf.history.size() > max_history_) {
            const auto erase_count = buf.history.size() - max_history_;
            buf.history.erase(
                buf.history.begin(),
                buf.history.begin() + static_cast<long>(erase_count));
        }
    }
}

std::size_t TelemetryStore::maxHistory() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return max_history_;
}

void TelemetryStore::clear()
{
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& [_, buf] : buffers_) {
        buf.latest.reset();
        buf.history.clear();
    }
}

void TelemetryStore::clearDomain(Domain domain)
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return;

    it->second.latest.reset();
    it->second.history.clear();
}

void TelemetryStore::push(const TelemetrySample& sample)
{
    if (!sample.valid) return;

    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(sample.domain);
    if (it == buffers_.end()) return;

    auto& buf = it->second;
    buf.latest = sample;
    buf.history.push_back(sample);

    if (buf.history.size() > max_history_) {
        const auto overflow = buf.history.size() - max_history_;
        buf.history.erase(
            buf.history.begin(),
            buf.history.begin() + static_cast<long>(overflow));
    }
}

std::optional<TelemetrySample> TelemetryStore::latest(Domain domain) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return std::nullopt;
    return it->second.latest;
}

std::optional<TelemetrySample> TelemetryStore::getLatestSample(Domain domain) const
{
    return latest(domain);
}

std::vector<TelemetrySample> TelemetryStore::history(
    Domain domain,
    std::size_t max_points) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return {};

    return downsampleUniform_(
               it->second.history,
               std::max<std::size_t>(1, max_points));
}

std::vector<TelemetrySample> TelemetryStore::historyRange(
    Domain domain,
    std::size_t start_index,
    std::size_t end_index) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return {};

    const auto& hist = it->second.history;
    if (start_index >= hist.size()) return {};

    const std::size_t actual_end = std::min(end_index, hist.size());
    if (actual_end <= start_index) return {};

    return std::vector<TelemetrySample>(
               hist.begin() + static_cast<long>(start_index),
               hist.begin() + static_cast<long>(actual_end));
}

std::size_t TelemetryStore::size(Domain domain) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return 0;
    return it->second.history.size();
}

std::size_t TelemetryStore::sampleCount(Domain domain) const
{
    return size(domain);
}

std::optional<TelemetrySample> TelemetryStore::getSample(
    Domain domain,
    std::size_t index) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return std::nullopt;

    const auto& hist = it->second.history;
    if (index >= hist.size()) return std::nullopt;

    return hist[index];
}

std::vector<double> TelemetryStore::extractSignal(
    Domain domain,
    TelemetrySignalType signal,
    std::size_t start_index,
    std::size_t count) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return {};

    const auto& hist = it->second.history;
    if (start_index >= hist.size()) return {};

    const std::size_t actual_count = (count == 0) ?
                                     (hist.size() - start_index) :
                                     std::min(count, hist.size() - start_index);

    std::vector<double> result;
    result.reserve(actual_count);

    for (std::size_t i = start_index; i < start_index + actual_count; ++i) {
        result.push_back(extractSignalValue_(hist[i], signal));
    }

    return result;
}

std::vector<double> TelemetryStore::extractSignalDownsampled(
    Domain domain,
    TelemetrySignalType signal,
    std::size_t max_points,
    std::size_t start_index) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return {};

    const auto& hist = it->second.history;
    if (start_index >= hist.size()) return {};

    // Extract subset first
    std::vector<TelemetrySample> subset(
        hist.begin() + static_cast<long>(start_index),
        hist.end());

    // Downsample
    const auto downsampled = downsampleUniform_(subset, max_points);

    // Extract signal values
    std::vector<double> result;
    result.reserve(downsampled.size());

    for (const auto& sample : downsampled) {
        result.push_back(extractSignalValue_(sample, signal));
    }

    return result;
}

std::optional<TelemetryStore::SignalStats> TelemetryStore::computeSignalStats(
    Domain domain,
    TelemetrySignalType signal,
    std::size_t start_index,
    std::size_t count) const
{
    const auto values = extractSignal(domain, signal, start_index, count);
    if (values.empty()) return std::nullopt;

    SignalStats stats;
    stats.count = values.size();

    // Min/Max
    const auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
    stats.min = *min_it;
    stats.max = *max_it;

    // Mean
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    stats.mean = sum / static_cast<double>(values.size());

    // Standard deviation
    double sq_sum = 0.0;
    for (const double v : values) {
        const double diff = v - stats.mean;
        sq_sum += diff * diff;
    }
    stats.stddev = std::sqrt(sq_sum / static_cast<double>(values.size()));

    return stats;
}

std::vector<TelemetrySample> TelemetryStore::getRecentSamples(
    Domain domain,
    double time_window_sec) const
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = buffers_.find(domain);
    if (it == buffers_.end()) return {};

    const auto& hist = it->second.history;
    if (hist.empty()) return {};

    // Get the latest timestamp
    const auto latest_time = hist.back().t;

    // Find samples within the time window
    std::vector<TelemetrySample> result;
    for (auto rit = hist.rbegin(); rit != hist.rend(); ++rit) {
        const auto time_diff = std::chrono::duration<double>(
                                   latest_time - rit->t).count();

        if (time_diff > time_window_sec) break;

        result.push_back(*rit);
    }

    // Reverse to maintain chronological order
    std::reverse(result.begin(), result.end());

    return result;
}

std::vector<TelemetrySample> TelemetryStore::downsampleUniform_(
    const std::vector<TelemetrySample>& in,
    std::size_t max_points)
{
    if (in.size() <= max_points) return in;
    if (max_points == 0) return {};

    std::vector<TelemetrySample> out;
    out.reserve(max_points);

    const double step = static_cast<double>(in.size() - 1) /
                        static_cast<double>(max_points - 1);

    for (std::size_t i = 0; i < max_points; ++i) {
        const std::size_t idx = static_cast<std::size_t>(i * step);
        out.push_back(in[idx]);
    }

    return out;
}

double TelemetryStore::extractSignalValue_(
    const TelemetrySample& sample,
    TelemetrySignalType signal)
{
    switch (signal) {
    case TelemetrySignalType::Angle:
        return sample.arm_angle_rad;
    case TelemetrySignalType::MotorSpeed:
        return sample.motor_speed_rad_s;
    case TelemetrySignalType::PwmFeedback:
        return static_cast<double>(sample.pwm_us);
    case TelemetrySignalType::AngleRef:
        return sample.ref_angle_rad;
    default:
        return 0.0;
    }
}

} // namespace prop_arm::models
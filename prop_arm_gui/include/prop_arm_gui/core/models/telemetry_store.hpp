#pragma once

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/domain_buffer.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/telemetry_signal_type.hpp"

#include <cstddef>
#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace prop_arm::models {

class TelemetryStore final {
public:
    explicit TelemetryStore(std::size_t max_history = 10000);

    ~TelemetryStore() = default;

    // Prevent copy/move
    TelemetryStore(const TelemetryStore&) = delete;
    TelemetryStore& operator=(const TelemetryStore&) = delete;
    TelemetryStore(TelemetryStore&&) = delete;
    TelemetryStore& operator=(TelemetryStore&&) = delete;

    // Configuration
    void setMaxHistory(std::size_t max_history);
    [[nodiscard]] std::size_t maxHistory() const;

    // Data management
    void clear();
    void clearDomain(Domain domain);
    void push(const TelemetrySample& sample);

    // Query - Latest
    [[nodiscard]] std::optional<TelemetrySample> latest(Domain domain) const;

    // Query - History
    [[nodiscard]] std::vector<TelemetrySample> history(
        Domain domain,
        std::size_t max_points = 1000) const;

    [[nodiscard]] std::vector<TelemetrySample> historyRange(
        Domain domain,
        std::size_t start_index,
        std::size_t end_index) const;

    // Query - Size
    [[nodiscard]] std::size_t size(Domain domain) const;
    [[nodiscard]] std::size_t sampleCount(Domain domain) const; // Alias for size()

    // Query - Individual samples
    [[nodiscard]] std::optional<TelemetrySample> getSample(
        Domain domain,
        std::size_t index) const;

    [[nodiscard]] std::optional<TelemetrySample> getLatestSample(Domain domain) const;

    // Query - Signal extraction
    [[nodiscard]] std::vector<double> extractSignal(
        Domain domain,
        TelemetrySignalType signal,
        std::size_t start_index = 0,
        std::size_t count = 0) const; // count=0 means all

    [[nodiscard]] std::vector<double> extractSignalDownsampled(
        Domain domain,
        TelemetrySignalType signal,
        std::size_t max_points,
        std::size_t start_index = 0) const;

    // Statistics
    struct SignalStats {
        double min{0.0};
        double max{0.0};
        double mean{0.0};
        double stddev{0.0};
        std::size_t count{0};
    };

    [[nodiscard]] std::optional<SignalStats> computeSignalStats(
        Domain domain,
        TelemetrySignalType signal,
        std::size_t start_index = 0,
        std::size_t count = 0) const;

    // Time-based queries
    [[nodiscard]] std::vector<TelemetrySample> getRecentSamples(
        Domain domain,
        double time_window_sec) const;

private:
    [[nodiscard]] static std::vector<TelemetrySample> downsampleUniform_(
        const std::vector<TelemetrySample>& in,
        std::size_t max_points);

    [[nodiscard]] static double extractSignalValue_(
        const TelemetrySample& sample,
        TelemetrySignalType signal);

private:
    mutable std::mutex mtx_;
    std::size_t max_history_;
    std::map<Domain, DomainBuffer> buffers_;
};

} // namespace prop_arm::models
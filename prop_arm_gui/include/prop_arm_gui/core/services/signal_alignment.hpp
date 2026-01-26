#pragma once

#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/telemetry_signal_type.hpp"

#include <chrono>
#include <cstddef>
#include <optional>
#include <vector>

namespace prop_arm::services {

struct AlignedSignals final {
    std::vector<double> t_sec;   // from 0..T
    std::vector<double> real_y;
    std::vector<double> sim_y;
};

struct ErrorMetrics final {
    double mse{0.0};
    double rmse{0.0};
    double max_abs{0.0};
    double mean_abs{0.0};
    double correlation{0.0};
    std::size_t n{0};
};

class SignalAlignment final {
public:
    // Extract a signal from samples and resample both domains to uniform dt using linear interpolation.
    // Returns nullopt if not enough samples.
    static std::optional<AlignedSignals> alignAndResample(
        const std::vector<prop_arm::models::TelemetrySample>& real_samples,
        const std::vector<prop_arm::models::TelemetrySample>& sim_samples,
        prop_arm::models::TelemetrySignalType signal,
        double dt_sec);

    static ErrorMetrics computeError(const AlignedSignals& a);

private:
    struct Series final {
        std::vector<double> t;
        std::vector<double> y;
    };

    static Series buildSeries_(
        const std::vector<prop_arm::models::TelemetrySample>& samples,
        prop_arm::models::TelemetrySignalType signal);

    static double sampleToValue_(
        const prop_arm::models::TelemetrySample& s,
        prop_arm::models::TelemetrySignalType signal);

    static std::vector<double> linspace_(double t0, double t1, double dt);
    static std::vector<double> interpLinear_(const Series& s, const std::vector<double>& t_out);

    static double safeCorrelation_(const std::vector<double>& a, const std::vector<double>& b);
};

}  // namespace prop_arm::services

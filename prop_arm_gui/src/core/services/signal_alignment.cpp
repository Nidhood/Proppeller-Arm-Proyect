#include "prop_arm_gui/core/services/signal_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace prop_arm::services {

using prop_arm::models::TelemetrySample;
using prop_arm::models::TelemetrySignalType;

SignalAlignment::Series SignalAlignment::buildSeries_(
    const std::vector<TelemetrySample>& samples,
    TelemetrySignalType signal)
{
    Series s;
    s.t.reserve(samples.size());
    s.y.reserve(samples.size());

    if (samples.empty()) return s;

    const auto t0 = samples.front().t;

    for (const auto& smp : samples) {
        const double tsec = std::chrono::duration<double>(smp.t - t0).count();
        s.t.push_back(tsec);
        s.y.push_back(sampleToValue_(smp, signal));
    }
    return s;
}

double SignalAlignment::sampleToValue_(const TelemetrySample& s, TelemetrySignalType signal)
{
    switch (signal) {
    case TelemetrySignalType::Angle:
        return s.arm_angle_rad;
    case TelemetrySignalType::MotorSpeed:
        return s.motor_speed_rad_s;
    case TelemetrySignalType::PwmFeedback:
        return static_cast<double>(s.pwm_us);
    case TelemetrySignalType::AngleRef:
        return s.ref_angle_rad;
    default:
        return 0.0;
    }
}

std::vector<double> SignalAlignment::linspace_(double t0, double t1, double dt)
{
    if (dt <= 0.0) return {};
    if (t1 <= t0) return {t0};

    const std::size_t n = static_cast<std::size_t>(std::floor((t1 - t0) / dt)) + 1;
    std::vector<double> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(t0 + static_cast<double>(i) * dt);
    }
    // ensure last is <= t1
    if (!out.empty() && out.back() > t1) out.back() = t1;
    return out;
}

std::vector<double> SignalAlignment::interpLinear_(const Series& s, const std::vector<double>& t_out)
{
    std::vector<double> y_out;
    y_out.reserve(t_out.size());
    if (s.t.size() < 2) return y_out;

    std::size_t j = 0;
    for (double tq : t_out) {
        while (j + 1 < s.t.size() && s.t[j + 1] < tq) ++j;

        if (tq <= s.t.front()) {
            y_out.push_back(s.y.front());
            continue;
        }
        if (tq >= s.t.back()) {
            y_out.push_back(s.y.back());
            continue;
        }

        const double t0 = s.t[j];
        const double t1 = s.t[j + 1];
        const double y0 = s.y[j];
        const double y1 = s.y[j + 1];

        const double alpha = (tq - t0) / (t1 - t0);
        y_out.push_back(y0 + alpha * (y1 - y0));
    }
    return y_out;
}

double SignalAlignment::safeCorrelation_(const std::vector<double>& a, const std::vector<double>& b)
{
    if (a.size() != b.size() || a.size() < 2) return 0.0;

    const double mean_a = std::accumulate(a.begin(), a.end(), 0.0) / static_cast<double>(a.size());
    const double mean_b = std::accumulate(b.begin(), b.end(), 0.0) / static_cast<double>(b.size());

    double num = 0.0;
    double da2 = 0.0;
    double db2 = 0.0;

    for (std::size_t i = 0; i < a.size(); ++i) {
        const double da = a[i] - mean_a;
        const double db = b[i] - mean_b;
        num += da * db;
        da2 += da * da;
        db2 += db * db;
    }

    const double den = std::sqrt(da2 * db2);
    if (den <= 1e-12) return 0.0;
    return num / den;
}

std::optional<AlignedSignals> SignalAlignment::alignAndResample(
    const std::vector<TelemetrySample>& real_samples,
    const std::vector<TelemetrySample>& sim_samples,
    TelemetrySignalType signal,
    double dt_sec)
{
    if (real_samples.size() < 2 || sim_samples.size() < 2) return std::nullopt;

    // Build relative series (each relative to its own first sample)
    auto real_s = buildSeries_(real_samples, signal);
    auto sim_s  = buildSeries_(sim_samples, signal);

    // Common overlap window
    const double t0 = std::max(real_s.t.front(), sim_s.t.front());
    const double t1 = std::min(real_s.t.back(),  sim_s.t.back());
    if (t1 - t0 <= 2.0 * dt_sec) return std::nullopt;

    auto t = linspace_(t0, t1, dt_sec);

    AlignedSignals out;
    out.t_sec = t;
    out.real_y = interpLinear_(real_s, t);
    out.sim_y  = interpLinear_(sim_s,  t);

    if (out.real_y.size() != out.sim_y.size() || out.real_y.size() < 2) return std::nullopt;
    return out;
}

ErrorMetrics SignalAlignment::computeError(const AlignedSignals& a)
{
    ErrorMetrics m;
    const std::size_t n = std::min(a.real_y.size(), a.sim_y.size());
    if (n == 0) return m;

    double se = 0.0;
    double max_abs = 0.0;
    double sum_abs = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        const double e = a.real_y[i] - a.sim_y[i];
        const double ae = std::abs(e);
        se += e * e;
        sum_abs += ae;
        max_abs = std::max(max_abs, ae);
    }

    m.n = n;
    m.mse = se / static_cast<double>(n);
    m.rmse = std::sqrt(m.mse);
    m.max_abs = max_abs;
    m.mean_abs = sum_abs / static_cast<double>(n);
    m.correlation = safeCorrelation_(a.real_y, a.sim_y);
    return m;
}

}  // namespace prop_arm::services

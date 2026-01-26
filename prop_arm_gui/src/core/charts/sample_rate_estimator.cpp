#include "prop_arm_gui/core/charts/sample_rate_estimator.hpp"
#include <algorithm>
#include <cmath>

namespace prop_arm::core::charts {

namespace {

inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

inline void updateDtEma(double dt, double& ema_dt, bool& has_ema) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;

    dt = clampd(dt, 1e-5, 0.5);

    if (!has_ema) {
        ema_dt = dt;
        has_ema = true;
        return;
    }

    constexpr double alpha = 0.08;
    ema_dt = alpha * dt + (1.0 - alpha) * ema_dt;
}

inline int clampi(int v, int lo, int hi) {
    return std::max(lo, std::min(v, hi));
}

} // namespace

SampleRateEstimator::State& SampleRateEstimator::getState(prop_arm::models::Domain d) {
    switch (d) {
    case prop_arm::models::Domain::Real:
        return real_;
    case prop_arm::models::Domain::Sim:
        return sim_;
    case prop_arm::models::Domain::Ref:
        return ref_;
    default:
        return real_;
    }
}

const SampleRateEstimator::State& SampleRateEstimator::getState(prop_arm::models::Domain d) const {
    switch (d) {
    case prop_arm::models::Domain::Real:
        return real_;
    case prop_arm::models::Domain::Sim:
        return sim_;
    case prop_arm::models::Domain::Ref:
        return ref_;
    default:
        return real_;
    }
}

void SampleRateEstimator::observe(prop_arm::models::Domain d, double t) {
    if (!std::isfinite(t)) return;

    auto& st = getState(d);

    if (std::isfinite(st.last_t)) {
        const double dt = t - st.last_t;
        updateDtEma(dt, st.dt_ema, st.has_dt);
    }

    st.last_t = t;
}

std::optional<double> SampleRateEstimator::dtEma(prop_arm::models::Domain d) const {
    const auto& st = getState(d);

    if (!st.has_dt || !(st.dt_ema > 0.0) || !std::isfinite(st.dt_ema)) {
        return std::nullopt;
    }

    return st.dt_ema;
}

int SampleRateEstimator::recommendedMaxPoints(double window_s, double dt) {
    if (!(window_s > 0.0) || !(dt > 0.0) || !std::isfinite(window_s) || !std::isfinite(dt)) {
        return 2000;
    }

    constexpr int kMinPoints = 200;
    constexpr int kMaxPointsCap = 20000;

    const int base   = static_cast<int>(std::ceil(window_s / dt)) + 10;
    const int scaled = static_cast<int>(std::ceil(base * 1.2));

    return clampi(scaled, kMinPoints, kMaxPointsCap);
}

void SampleRateEstimator::clear() {
    real_ = State{};
    sim_ = State{};
    ref_ = State{};
}

} // namespace prop_arm::core::charts
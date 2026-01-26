#include "prop_arm_gui/core/charts/chart_window.hpp"

#include <algorithm>
#include <cmath>

namespace prop_arm::core::charts {

namespace {
constexpr double kTiny = 1e-12;
} // namespace

void ChartWindow::setLimits(Limits l) {
    limits_.window_s   = std::max(0.5, l.window_s);
    limits_.max_points = std::max(1, l.max_points);
    enforce_();
}

std::deque<QPointF>& ChartWindow::buf_(prop_arm::models::Domain d) {
    switch (d) {
    case prop_arm::models::Domain::Real:
        return real_;
    case prop_arm::models::Domain::Sim:
        return sim_;
    case prop_arm::models::Domain::Ref:
        return ref_;
    }
    return real_; // Fallback (should never happen)
}

const std::deque<QPointF>& ChartWindow::buf_(prop_arm::models::Domain d) const {
    switch (d) {
    case prop_arm::models::Domain::Real:
        return real_;
    case prop_arm::models::Domain::Sim:
        return sim_;
    case prop_arm::models::Domain::Ref:
        return ref_;
    }
    return real_;
}

void ChartWindow::append(double t, double y, prop_arm::models::Domain d) {
    if (!std::isfinite(t) || !std::isfinite(y)) return;

    auto& b = buf_(d);
    b.emplace_back(QPointF(t, y));
    enforce_();
}

void ChartWindow::clear() {
    real_.clear();
    sim_.clear();
    ref_.clear();
    span_ = {};
}

void ChartWindow::enforce_() {
    double t_max = 0.0;
    bool has = false;

    auto consider = [&](const std::deque<QPointF>& b) {
        if (!b.empty()) {
            t_max = std::max(t_max, b.back().x());
            has = true;
        }
    };

    consider(real_);
    consider(sim_);
    consider(ref_);

    if (!has) {
        span_.t_min = 0.0;
        span_.t_max = 0.0;
        return;
    }

    const double window_s = std::max(0.5, limits_.window_s);
    const double t_min = std::max(0.0, t_max - window_s);

    auto trim = [&](std::deque<QPointF>& b) {
        while (!b.empty() && b.front().x() < t_min) b.pop_front();
        while (static_cast<int>(b.size()) > limits_.max_points) b.pop_front();
    };

    trim(real_);
    trim(sim_);
    trim(ref_);

    span_.t_min = t_min;
    span_.t_max = t_max;

    if (span_.t_max - span_.t_min < kTiny) {
        span_.t_min = std::max(0.0, span_.t_max - window_s);
    }
}

} // namespace prop_arm::core::charts

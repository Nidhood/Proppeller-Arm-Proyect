#include "prop_arm_gui/core/charts/hover_interpolator.hpp"

#include <algorithm>
#include <cmath>

namespace prop_arm::core::charts {

namespace {
constexpr double kTiny = 1e-12;
} // namespace

std::optional<QPointF> HoverInterpolator::yAt(const std::deque<QPointF>& buf, double t) {
    if (buf.empty() || !std::isfinite(t)) return std::nullopt;

    const double t0 = buf.front().x();
    const double tN = buf.back().x();
    if (t < t0 || t > tN) return std::nullopt;

    auto it = std::lower_bound(
                  buf.begin(), buf.end(), t,
    [](const QPointF& p, double tx) {
        return p.x() < tx;
    });

    if (it == buf.begin()) return QPointF(t, it->y());
    if (it == buf.end())   return QPointF(t, buf.back().y());

    const auto prev = std::prev(it);
    const double t1 = prev->x();
    const double t2 = it->x();
    const double v1 = prev->y();
    const double v2 = it->y();

    const double denom = (t2 - t1);
    if (std::abs(denom) < kTiny) return QPointF(t, v2);

    const double a = std::clamp((t - t1) / denom, 0.0, 1.0);
    return QPointF(t, v1 + a * (v2 - v1));
}

} // namespace prop_arm::core::charts

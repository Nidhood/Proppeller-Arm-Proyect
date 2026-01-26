#pragma once

#include <deque>
#include <optional>
#include <QPointF>

namespace prop_arm::core::charts {

class HoverInterpolator {
public:
    static std::optional<QPointF> yAt(const std::deque<QPointF>& buf, double t);
};

} // namespace prop_arm::core::charts

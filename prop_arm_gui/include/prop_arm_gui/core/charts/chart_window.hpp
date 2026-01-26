#pragma once

#include <deque>
#include <QPointF>

#include "prop_arm_gui/core/models/domain.hpp"

namespace prop_arm::core::charts {

class ChartWindow {
public:
    struct Limits {
        double window_s{10.0};
        int max_points{2000};
    };

    struct Span {
        double t_min{0.0};
        double t_max{0.0};
    };

    void setLimits(Limits l);

    // Supports Real, Sim, Ref.
    void append(double t, double y, prop_arm::models::Domain d);

    [[nodiscard]] Span span() const {
        return span_;
    }

    [[nodiscard]] const std::deque<QPointF>& real() const {
        return real_;
    }
    [[nodiscard]] const std::deque<QPointF>& sim()  const {
        return sim_;
    }
    [[nodiscard]] const std::deque<QPointF>& ref()  const {
        return ref_;
    }

    void clear();

private:
    void enforce_();

    std::deque<QPointF>& buf_(prop_arm::models::Domain d);
    const std::deque<QPointF>& buf_(prop_arm::models::Domain d) const;

private:
    Limits limits_{};
    std::deque<QPointF> real_;
    std::deque<QPointF> sim_;
    std::deque<QPointF> ref_;
    Span span_{};
};

} // namespace prop_arm::core::charts

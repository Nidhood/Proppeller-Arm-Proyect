#pragma once

#include <deque>
#include <QPointF>
#include <QVector>

namespace prop_arm::core::charts {

class CurveBuilder {
public:
    enum class Mode { Raw, MonotoneCubic };

    void setMode(Mode m) {
        mode_ = m;
    }
    void setSegmentsPerInterval(int n);

    void build(const std::deque<QPointF>& src, QVector<QPointF>& out) const;

private:
    Mode mode_{Mode::MonotoneCubic};
    int segments_{3};
};

} // namespace prop_arm::core::charts

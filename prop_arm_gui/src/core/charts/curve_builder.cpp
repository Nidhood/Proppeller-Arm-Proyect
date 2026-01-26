#include "prop_arm_gui/core/charts/curve_builder.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace prop_arm::core::charts {

namespace {
constexpr double kTiny = 1e-12;

inline int clampi(int v, int lo, int hi) {
    return std::max(lo, std::min(v, hi));
}

// Fritsch-Carlson monotone cubic slopes (prevents overshoot).
inline void monotoneSlopes(const std::vector<double>& x,
                           const std::vector<double>& y,
                           std::vector<double>& m) {
    const int n = static_cast<int>(x.size());
    m.assign(n, 0.0);
    if (n < 2) return;

    std::vector<double> d(n - 1, 0.0);
    for (int i = 0; i < n - 1; ++i) {
        const double h = x[i + 1] - x[i];
        if (std::abs(h) < kTiny) d[i] = 0.0;
        else d[i] = (y[i + 1] - y[i]) / h;
    }

    m[0]     = d[0];
    m[n - 1] = d[n - 2];

    for (int i = 1; i < n - 1; ++i) {
        if (d[i - 1] * d[i] <= 0.0) {
            m[i] = 0.0;
        } else {
            const double w1 = 2.0 * (x[i + 1] - x[i]) + (x[i] - x[i - 1]);
            const double w2 = (x[i + 1] - x[i]) + 2.0 * (x[i] - x[i - 1]);
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
        }
    }

    // Safety limiter
    for (int i = 0; i < n - 1; ++i) {
        if (std::abs(d[i]) < kTiny) {
            m[i] = 0.0;
            m[i + 1] = 0.0;
            continue;
        }
        const double a = m[i] / d[i];
        const double b = m[i + 1] / d[i];
        const double s = a * a + b * b;
        if (s > 9.0) {
            const double t = 3.0 / std::sqrt(s);
            m[i]     = t * a * d[i];
            m[i + 1] = t * b * d[i];
        }
    }
}

inline double hermite(double y0, double y1, double m0, double m1, double h, double u) {
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h00 =  2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 =        u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 =        u3 -       u2;
    return h00 * y0 + h10 * (h * m0) + h01 * y1 + h11 * (h * m1);
}
} // namespace

void CurveBuilder::setSegmentsPerInterval(int n) {
    segments_ = clampi(n, 1, 20);
}

void CurveBuilder::build(const std::deque<QPointF>& src, QVector<QPointF>& out) const {
    out.clear();
    const int n = static_cast<int>(src.size());
    if (n <= 0) return;

    if (mode_ == Mode::Raw || n < 3) {
        out.reserve(n);
        for (const auto& p : src) out.push_back(p);
        return;
    }

    std::vector<double> x;
    std::vector<double> y;
    x.reserve(n);
    y.reserve(n);

    for (const auto& p : src) {
        x.push_back(p.x());
        y.push_back(p.y());
    }

    std::vector<double> m;
    monotoneSlopes(x, y, m);

    const int seg = clampi(segments_, 1, 20);
    out.reserve((n - 1) * seg + 1);

    for (int i = 0; i < n - 1; ++i) {
        const double x0 = x[i];
        const double x1 = x[i + 1];
        const double h = x1 - x0;
        if (std::abs(h) < kTiny) continue;

        const double y0 = y[i];
        const double y1 = y[i + 1];
        const double m0 = m[i];
        const double m1 = m[i + 1];

        for (int s = 0; s < seg; ++s) {
            const double u = static_cast<double>(s) / static_cast<double>(seg);
            const double xs = x0 + u * h;
            const double ys = hermite(y0, y1, m0, m1, h, u);
            out.push_back(QPointF(xs, ys));
        }
    }

    out.push_back(QPointF(x.back(), y.back()));
}

} // namespace prop_arm::core::charts

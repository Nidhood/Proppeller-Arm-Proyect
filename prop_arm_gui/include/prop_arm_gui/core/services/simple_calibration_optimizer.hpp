#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace prop_arm::services {

struct ParamBound {
    double min{0.0};
    double max{0.0};
};

inline double clamp(double v, const ParamBound& b) {
    return std::min(std::max(v, b.min), b.max);
}

class CoordinateDescent final {
public:
    using Objective = std::function<double(const std::vector<double>&)>;

    struct Settings {
        std::size_t max_iterations{30};
        double initial_step{0.15};      // relative step (15%)
        double min_step{1e-4};          // relative minimum step
        double shrink{0.5};             // step *= shrink when no improvement
        double improvement_eps{1e-6};   // consider improvement if delta > eps
    };

    CoordinateDescent(Settings s, std::vector<ParamBound> bounds)
        : s_(s), bounds_(std::move(bounds)) {}

    struct Result {
        std::vector<double> best_x;
        double best_cost{std::numeric_limits<double>::infinity()};
        std::size_t iters{0};
    };

    [[nodiscard]] Result minimize(
        const std::vector<double>& x0,
        const Objective& f) const
    {
        Result r;
        r.best_x = x0;
        project_(r.best_x);

        r.best_cost = f(r.best_x);

        double step = s_.initial_step;
        for (std::size_t it = 0; it < s_.max_iterations; ++it) {
            bool improved = false;

            for (std::size_t k = 0; k < r.best_x.size(); ++k) {
                const double xk = r.best_x[k];

                const double span = std::max(1e-12, bounds_[k].max - bounds_[k].min);
                const double delta = step * span;

                // Try +delta
                auto xp = r.best_x;
                xp[k] = clamp(xk + delta, bounds_[k]);
                const double cp = f(xp);

                // Try -delta
                auto xm = r.best_x;
                xm[k] = clamp(xk - delta, bounds_[k]);
                const double cm = f(xm);

                if (cp + s_.improvement_eps < r.best_cost && cp <= cm) {
                    r.best_cost = cp;
                    r.best_x = std::move(xp);
                    improved = true;
                } else if (cm + s_.improvement_eps < r.best_cost) {
                    r.best_cost = cm;
                    r.best_x = std::move(xm);
                    improved = true;
                }
            }

            r.iters = it + 1;

            if (!improved) {
                step *= s_.shrink;
                if (step < s_.min_step) break;
            }
        }

        return r;
    }

private:
    void project_(std::vector<double>& x) const {
        const std::size_t n = std::min(x.size(), bounds_.size());
        for (std::size_t i = 0; i < n; ++i) {
            x[i] = clamp(x[i], bounds_[i]);
        }
    }

private:
    Settings s_;
    std::vector<ParamBound> bounds_;
};

}  // namespace prop_arm::services

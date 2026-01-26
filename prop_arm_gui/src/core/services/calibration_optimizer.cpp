#include "prop_arm_gui/core/services/calibration_optimizer.hpp"

#include <algorithm>
#include <cmath>

namespace prop_arm::services {

static double clampTo(double v, const ParamBound& b) {
    return std::min(std::max(v, b.lo), b.hi);
}

OptimizerResult CoordinateDescentOptimizer::optimize(
    const std::vector<double>& x0,
    const std::vector<ParamBound>& bounds,
    const OptimizerConfig& cfg,
    const CostFn& cost_fn,
    const std::function<void(double, double)>& on_progress)
{
    OptimizerResult r;
    r.best_x = x0;

    // Clamp initial
    for (std::size_t i = 0; i < r.best_x.size(); ++i) {
        r.best_x[i] = clampTo(r.best_x[i], bounds[i]);
    }

    r.best_cost = cost_fn(r.best_x);
    r.eval_count++;

    std::vector<double> step(r.best_x.size(), 0.0);
    for (std::size_t i = 0; i < step.size(); ++i) {
        const double span = std::max(1e-9, bounds[i].hi - bounds[i].lo);
        step[i] = std::max(cfg.min_step_rel * span, cfg.step_scale * std::max(std::abs(r.best_x[i]), 1.0));
    }

    for (std::size_t it = 0; it < cfg.max_iterations; ++it) {
        bool improved_any = false;

        for (std::size_t k = 0; k < r.best_x.size(); ++k) {
            const double base = r.best_x[k];

            // Try +step
            auto x_plus = r.best_x;
            x_plus[k] = clampTo(base + step[k], bounds[k]);
            const double c_plus = cost_fn(x_plus);
            r.eval_count++;

            // Try -step
            auto x_minus = r.best_x;
            x_minus[k] = clampTo(base - step[k], bounds[k]);
            const double c_minus = cost_fn(x_minus);
            r.eval_count++;

            // Accept best move if improves
            if (c_plus < r.best_cost || c_minus < r.best_cost) {
                if (c_plus <= c_minus) {
                    r.best_x = std::move(x_plus);
                    r.best_cost = c_plus;
                } else {
                    r.best_x = std::move(x_minus);
                    r.best_cost = c_minus;
                }
                improved_any = true;
            } else {
                // Reduce step on this coordinate
                step[k] *= 0.5;
            }
        }

        if (on_progress) {
            const double p = (cfg.max_iterations <= 1) ? 1.0 : (static_cast<double>(it + 1) / static_cast<double>(cfg.max_iterations));
            on_progress(p, r.best_cost);
        }

        // Stop if no improvement and all steps small
        const bool steps_small = std::all_of(step.begin(), step.end(), [&](double s) {
            return s <= cfg.min_step_rel;
        });
        if (!improved_any && steps_small) break;
    }

    return r;
}

}  // namespace prop_arm::services

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace prop_arm::services {

struct ParamBound final {
    double lo{0.0};
    double hi{0.0};
};

struct OptimizerConfig final {
    std::size_t max_iterations{10};
    double step_scale{0.10};        // relative step (10%)
    double min_step_rel{1e-3};      // stop when steps get too small
};

struct OptimizerResult final {
    std::vector<double> best_x;
    double best_cost{0.0};
    std::size_t eval_count{0};
};

class CoordinateDescentOptimizer final {
public:
    using CostFn = std::function<double(const std::vector<double>& x)>;

    static OptimizerResult optimize(
        const std::vector<double>& x0,
        const std::vector<ParamBound>& bounds,
        const OptimizerConfig& cfg,
        const CostFn& cost_fn,
        const std::function<void(double progress01, double best_cost)>& on_progress = {});
};

}  // namespace prop_arm::services

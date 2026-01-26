#pragma once
#include <optional>
#include <limits>
#include "prop_arm_gui/core/models/domain.hpp"

namespace prop_arm::core::charts {

class SampleRateEstimator {
public:
    void observe(prop_arm::models::Domain d, double t);

    [[nodiscard]] std::optional<double> dtEma(prop_arm::models::Domain d) const;

    static int recommendedMaxPoints(double window_s, double dt);

    void clear();

private:
    struct State {
        bool has_dt{false};
        double dt_ema{0.0};
        double last_t{std::numeric_limits<double>::quiet_NaN()};
    };

    State& getState(prop_arm::models::Domain d);
    const State& getState(prop_arm::models::Domain d) const;

    State real_{};
    State sim_{};
    State ref_{};
};

} // namespace prop_arm::core::charts
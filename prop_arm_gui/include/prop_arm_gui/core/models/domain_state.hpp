
#include <deque>
#include <qpoint.h>

namespace prop_arm::models {

struct DomainRuntime {
    bool live{true};

    // dt EMA
    bool has_dt{false};
    double dt_ema{0.0};
    double last_t{std::numeric_limits<double>::quiet_NaN()};

    // y filter
    bool has_y{false};
    double y_f{0.0};

    std::deque<QPointF> buf;
};

} // namespace prop_arm::models

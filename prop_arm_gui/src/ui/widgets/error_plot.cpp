#include "prop_arm_gui/ui/widgets/error_plot.hpp"

#include "prop_arm_gui/ui/widgets/chart_base.hpp"

#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace prop_arm::ui::widgets {

ErrorPlot::ErrorPlot(QWidget* parent)
    : QWidget(parent) {

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    chart_ = new ChartBase("" /*title*/, "abs error (rad)", this);
    chart_->setObjectName("chartWidget");

    chart_->setShowSim(false);
    chart_->setShowRef(false);
    chart_->setAutoRange(true);

    // Single-curve semantics
    chart_->setSeriesLabels("Error", "" /*sim*/, "" /*ref*/);

    layout->addWidget(chart_, 1);
}

void ErrorPlot::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    (void)limits_;
}

void ErrorPlot::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (domain == prop_arm::models::Domain::Real) real_live_ = live;
    if (domain == prop_arm::models::Domain::Sim)  sim_live_  = live;

    // Error curve is meaningful only when BOTH streams are live
    const bool err_live = real_live_ && sim_live_;
    chart_->setRealLive(err_live);
}

double ErrorPlot::toSecSinceStart_(const std::chrono::steady_clock::time_point& tp) {
    if (!t0_.has_value()) t0_ = tp;
    const auto dt = tp - *t0_;
    return std::chrono::duration<double>(dt).count();
}

void ErrorPlot::pushSample(const prop_arm::models::TelemetrySample& sample) {
    if (!sample.valid) return;
    if (!std::isfinite(sample.arm_angle_rad)) return;

    // Establish global time base on first valid sample
    (void)toSecSinceStart_(sample.t);

    if (sample.domain == prop_arm::models::Domain::Real) {
        have_real_ = true;
        tp_real_ = sample.t;
        a_real_ = sample.arm_angle_rad;
        tryEmitError_();
    } else if (sample.domain == prop_arm::models::Domain::Sim) {
        have_sim_ = true;
        tp_sim_ = sample.t;
        a_sim_ = sample.arm_angle_rad;
        tryEmitError_();
    }
}

void ErrorPlot::tryEmitError_() {
    if (!(real_live_ && sim_live_)) return;
    if (!(have_real_ && have_sim_)) return;

    const double dt = std::abs(std::chrono::duration<double>(tp_real_ - tp_sim_).count());
    if (dt > sync_tol_s_) return;

    const double err = std::fabs(a_real_ - a_sim_);
    const auto tp = (tp_real_ > tp_sim_) ? tp_real_ : tp_sim_;
    const double t = toSecSinceStart_(tp);

    // Plot uses the "real" series as the single error series
    chart_->append(t, err, prop_arm::models::Domain::Real);
}

void ErrorPlot::clear() {
    have_real_ = false;
    have_sim_ = false;
    t0_.reset();
    if (chart_) chart_->clear();
}

void ErrorPlot::applyTheme(const prop_arm::ui::style::ThemeSpec& spec) {
    if (chart_) chart_->applyTheme(spec);
}

} // namespace prop_arm::ui::widgets

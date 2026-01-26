#include "prop_arm_gui/ui/widgets/duty_plot.hpp"

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/ui/widgets/chart_base.hpp"

#include <QFrame>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace prop_arm::ui::widgets {

DutyPlot::DutyPlot(QWidget* parent)
    : QWidget(parent) {
    root_ = new QVBoxLayout(this);
    root_->setContentsMargins(0, 0, 0, 0);
    root_->setSpacing(0);

    chart_ = new ChartBase("Duty", "%", this);
    chart_->setShowSim(true);
    chart_->setAutoRange(false);
    chart_->setWindowSeconds(30.0);
    chart_->setMaxPoints(2200);

    // Default until limits arrive:
    chart_->setYRange(0.0, 100.0);

    frame_ = static_cast<QFrame*>(makeChartFrame_(chart_, "rtpChartFrame"));
    root_->addWidget(frame_, 1);
}

QWidget* DutyPlot::makeChartFrame_(QWidget* child, const char* object_name) {
    auto* frame = new QFrame();
    frame->setObjectName(object_name);
    frame->setFrameShape(QFrame::NoFrame);

    auto* l = new QVBoxLayout(frame);
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(0);
    l->addWidget(child, 1);

    return frame;
}

std::chrono::steady_clock::time_point DutyPlot::sampleTime_(const prop_arm::models::TelemetrySample& s) {
    if (s.t == std::chrono::steady_clock::time_point{}) {
        return std::chrono::steady_clock::now();
    }
    return s.t;
}

void DutyPlot::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    applyLimits_();
}

void DutyPlot::applyLimits_() {
    if (!limits_ || !chart_) return;

    // ✅ Use YAML limits.duty (e.g., 5..10)
    const auto& d = limits_->dutyLimits();
    chart_->setYRange(d.min, d.max);
}

void DutyPlot::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (!chart_) return;
    if (domain == prop_arm::models::Domain::Real) chart_->setRealLive(live);
    else                                          chart_->setSimLive(live);
}

double DutyPlot::dutyPercent_(std::uint16_t pwm_us) const {
    if (!limits_) return 0.0;

    // Input PWM range
    const auto& p = limits_->pwmLimits();
    const double lo = static_cast<double>(p.min);
    const double hi = static_cast<double>(p.max);

    // Output duty range (e.g., 5..10)
    const auto& d = limits_->dutyLimits();
    const double dmin = d.min;
    const double dmax = d.max;

    const double v = static_cast<double>(pwm_us);
    const double denom = std::max(1.0, (hi - lo));
    const double x = std::clamp((v - lo) / denom, 0.0, 1.0);

    // ✅ Map PWM→[dmin..dmax]
    return dmin + x * (dmax - dmin);
}

void DutyPlot::pushSample(const prop_arm::models::TelemetrySample& sample) {
    if (!sample.valid || !chart_) return;

    const auto ts = sampleTime_(sample);
    if (!have_t0_) {
        have_t0_ = true;
        t0_ = ts;
    }

    const double t_sec = std::chrono::duration<double>(ts - t0_).count();
    const double duty = dutyPercent_(sample.pwm_us);

    chart_->append(t_sec, duty, sample.domain);
}

void DutyPlot::clear() {
    have_t0_ = false;
    if (chart_) chart_->clear();
}

}  // namespace prop_arm::ui::widgets

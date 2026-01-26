#include "prop_arm_gui/ui/widgets/pwm_plot.hpp"

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/ui/widgets/chart_base.hpp"

#include <QFrame>
#include <QVBoxLayout>

namespace prop_arm::ui::widgets {

PwmPlot::PwmPlot(QWidget* parent)
    : QWidget(parent) {
    root_ = new QVBoxLayout(this);
    root_->setContentsMargins(0, 0, 0, 0);
    root_->setSpacing(0);

    chart_ = new ChartBase("PWM", "us", this);
    chart_->setShowSim(true);
    chart_->setIntegerYAxis(true);
    chart_->setAutoRange(false);
    chart_->setWindowSeconds(30.0);
    chart_->setMaxPoints(2200);

    frame_ = static_cast<QFrame*>(makeChartFrame_(chart_, "rtpChartFrame"));
    root_->addWidget(frame_, 1);
}

QWidget* PwmPlot::makeChartFrame_(QWidget* child, const char* object_name) {
    auto* frame = new QFrame();
    frame->setObjectName(object_name);
    frame->setFrameShape(QFrame::NoFrame);

    auto* l = new QVBoxLayout(frame);
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(0);
    l->addWidget(child, 1);

    return frame;
}

std::chrono::steady_clock::time_point PwmPlot::sampleTime_(const prop_arm::models::TelemetrySample& s) {
    if (s.t == std::chrono::steady_clock::time_point{}) {
        return std::chrono::steady_clock::now();
    }
    return s.t;
}

void PwmPlot::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    if (!limits_ || !chart_) return;

    const auto& r = limits_->pwmLimits();
    chart_->setYRange(static_cast<double>(r.min), static_cast<double>(r.max));
}

void PwmPlot::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (!chart_) return;
    if (domain == prop_arm::models::Domain::Real) chart_->setRealLive(live);
    else                                          chart_->setSimLive(live);
}

void PwmPlot::pushSample(const prop_arm::models::TelemetrySample& sample) {
    if (!sample.valid || !chart_) return;

    const auto ts = sampleTime_(sample);
    if (!have_t0_) {
        have_t0_ = true;
        t0_ = ts;
    }

    const double t_sec = std::chrono::duration<double>(ts - t0_).count();
    const double y = static_cast<double>(sample.pwm_us);

    chart_->append(t_sec, y, sample.domain);
}

void PwmPlot::clear() {
    have_t0_ = false;
    if (chart_) chart_->clear();
}

}  // namespace prop_arm::ui::widgets

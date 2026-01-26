#include "prop_arm_gui/ui/widgets/tracking_error_plot.hpp"

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/ui/widgets/chart_base.hpp"

#include <QVBoxLayout>
#include <algorithm>
#include <numbers>

namespace prop_arm::ui::widgets {

TrackingErrorPlot::TrackingErrorPlot(QWidget* parent)
    : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    chart_ = new ChartBase("Tracking error (Ref - Angle)", "deg", this);
    chart_->setShowSim(true);
    chart_->setIntegerYAxis(false);
    chart_->setAutoRange(false);
    chart_->setWindowSeconds(30.0);
    chart_->setMaxPoints(1200);

    // Safe default:
    chart_->setYRange(-90.0, 90.0);

    root->addWidget(chart_, 1);
}

std::chrono::steady_clock::time_point TrackingErrorPlot::sampleTime_(const prop_arm::models::TelemetrySample& s) {
    if (s.t == std::chrono::steady_clock::time_point{}) {
        return std::chrono::steady_clock::now();
    }
    return s.t;
}

void TrackingErrorPlot::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    applyLimits_();
}

void TrackingErrorPlot::applyLimits_() {
    if (!limits_ || !chart_) return;

    // ✅ Use YAML limits.error_tracking
    const auto& r = limits_->errorTrackingLimits();
    chart_->setYRange(r.min, r.max);
}

void TrackingErrorPlot::pushSample(const prop_arm::models::TelemetrySample& sample) {
    if (!chart_ || !sample.valid) return;

    const auto ts = sampleTime_(sample);
    if (!have_t0_) {
        have_t0_ = true;
        t0_ = ts;
    }

    const double t_sec = std::chrono::duration<double>(ts - t0_).count();

    const double arm_deg = sample.arm_angle_rad * 180.0 / std::numbers::pi;
    const double ref_deg = sample.ref_angle_rad * 180.0 / std::numbers::pi;
    const double err_deg = ref_deg - arm_deg;

    chart_->append(t_sec, err_deg, sample.domain);
}

void TrackingErrorPlot::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (!chart_) return;
    if (domain == prop_arm::models::Domain::Real) chart_->setRealLive(live);
    else                                          chart_->setSimLive(live);
}

void TrackingErrorPlot::clear() {
    have_t0_ = false;
    if (chart_) chart_->clear();
}

}  // namespace prop_arm::ui::widgets

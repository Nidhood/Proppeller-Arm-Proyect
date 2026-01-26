#include "prop_arm_gui/ui/widgets/angle_ref_plot.hpp"

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/ui/widgets/chart_base.hpp"

#include <QVBoxLayout>
#include <numbers>

namespace prop_arm::ui::widgets {

AngleRefPlot::AngleRefPlot(QWidget* parent)
    : QWidget(parent) {
    buildUi_();
}

void AngleRefPlot::buildUi_() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    chart_ = new ChartBase("Arm angle vs Ref", "deg", this);
    chart_->setShowSim(true);
    chart_->setShowRef(true);
    chart_->setRealLive(false);
    chart_->setSimLive(false);
    chart_->setRefLive(false);
    chart_->setWindowSeconds(30.0);
    chart_->setMaxPoints(800);
    applyLimits_();
    root->addWidget(chart_, 1);
}

std::chrono::steady_clock::time_point AngleRefPlot::sampleTime_(
    const prop_arm::models::TelemetrySample& s) {
    if (s.t == std::chrono::steady_clock::time_point{}) {
        return std::chrono::steady_clock::now();
    }
    return s.t;
}

void AngleRefPlot::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    applyLimits_();
}

void AngleRefPlot::applyLimits_() {
    if (!limits_ || !chart_) return;

    const auto& a = limits_->angleLimits();
    chart_->setYRange(a.min, a.max);
}

void AngleRefPlot::updateHeldRef_(prop_arm::models::Domain domain, double ref_deg) {
    if (domain == prop_arm::models::Domain::Real) {
        have_real_ref_ = true;
        have_ref_ = true;
        held_ref_deg_ = ref_deg;
        return;
    }

    if (domain == prop_arm::models::Domain::Sim) {
        if (!have_real_ref_) {
            have_ref_ = true;
            held_ref_deg_ = ref_deg;
        }
        return;
    }

    if (domain == prop_arm::models::Domain::Ref) {
        if (!have_real_ref_) {
            have_ref_ = true;
            held_ref_deg_ = ref_deg;
        }
        return;
    }
}

void AngleRefPlot::appendHeldRefAt_(double t_sec) {
    if (!chart_ || !have_ref_) return;
    chart_->appendRef(t_sec, held_ref_deg_);
}

void AngleRefPlot::pushSample(const prop_arm::models::TelemetrySample& sample) {
    if (!chart_ || !sample.valid) return;

    const auto ts = sampleTime_(sample);
    if (!have_t0_) {
        have_t0_ = true;
        t0_ = ts;
    }

    const double t_sec = std::chrono::duration<double>(ts - t0_).count();

    const double arm_deg = sample.arm_angle_rad * 180.0 / std::numbers::pi;
    const double ref_deg = sample.ref_angle_rad * 180.0 / std::numbers::pi;
    updateHeldRef_(sample.domain, ref_deg);
    if (sample.domain == prop_arm::models::Domain::Real ||
            sample.domain == prop_arm::models::Domain::Sim) {
        chart_->append(t_sec, arm_deg, sample.domain);
        appendHeldRefAt_(t_sec);
    }
}

void AngleRefPlot::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (!chart_) return;

    if (domain == prop_arm::models::Domain::Real) {
        live_real_ = live;
        chart_->setRealLive(live);
    } else if (domain == prop_arm::models::Domain::Sim) {
        live_sim_ = live;
        chart_->setSimLive(live);
    }

    chart_->setRefLive(live_real_ || live_sim_);
}

void AngleRefPlot::clear() {
    have_t0_ = false;
    t0_ = {};

    live_real_ = false;
    live_sim_ = false;

    have_real_ref_ = false;
    have_ref_ = false;
    held_ref_deg_ = 0.0;

    if (chart_) chart_->clear();
}

}  // namespace prop_arm::ui::widgets

#include "prop_arm_gui/ui/pages/dashboard_page.hpp"
#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/telemetry_store.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"

#include "prop_arm_gui/ui/widgets/angle_plot.hpp"
#include "prop_arm_gui/ui/widgets/duty_plot.hpp"
#include "prop_arm_gui/ui/widgets/pwm_plot.hpp"
#include "prop_arm_gui/ui/widgets/speed_plot.hpp"

#include <QGridLayout>
#include <QVBoxLayout>

namespace prop_arm::ui::pages {

DashboardPage::DashboardPage(QWidget* parent)
    : QWidget(parent) {
    buildUi_();
    setStreamLive(prop_arm::models::Domain::Real, true);
    setStreamLive(prop_arm::models::Domain::Sim, true);
    wireLimits_();
}

void DashboardPage::setTelemetryStore(prop_arm::models::TelemetryStore* store) noexcept {
    store_ = store;
}

void DashboardPage::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    wireLimits_();
}

void DashboardPage::wireLimits_() noexcept {
    if (!limits_) return;

    if (angle_plot_) angle_plot_->setLimitsRegistry(limits_);
    if (speed_plot_) speed_plot_->setLimitsRegistry(limits_);
    if (pwm_plot_)   pwm_plot_->setLimitsRegistry(limits_);
    if (duty_plot_)  duty_plot_->setLimitsRegistry(limits_);
}

void DashboardPage::buildUi_() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(16);
    auto* grid = new QGridLayout();
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setHorizontalSpacing(12);
    grid->setVerticalSpacing(12);
    angle_plot_ = new prop_arm::ui::widgets::AnglePlot(this);
    speed_plot_ = new prop_arm::ui::widgets::SpeedPlot(this);
    pwm_plot_   = new prop_arm::ui::widgets::PwmPlot(this);
    duty_plot_  = new prop_arm::ui::widgets::DutyPlot(this);
    grid->addWidget(angle_plot_, 0, 0);
    grid->addWidget(speed_plot_, 0, 1);
    grid->addWidget(pwm_plot_,   1, 0);
    grid->addWidget(duty_plot_,  1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    root->addLayout(grid, 1);
}

void DashboardPage::onTelemetry(const prop_arm::models::TelemetrySample& sample) {
    if (angle_plot_) angle_plot_->pushSample(sample);
    if (speed_plot_) speed_plot_->pushSample(sample);
    if (pwm_plot_)   pwm_plot_->pushSample(sample);
    if (duty_plot_)  duty_plot_->pushSample(sample);
}

void DashboardPage::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (angle_plot_) angle_plot_->setStreamLive(domain, live);
    if (speed_plot_) speed_plot_->setStreamLive(domain, live);
    if (pwm_plot_)   pwm_plot_->setStreamLive(domain, live);
    if (duty_plot_)  duty_plot_->setStreamLive(domain, live);
}

}  // namespace prop_arm::ui::pages

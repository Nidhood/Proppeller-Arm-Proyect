#pragma once

#include <QWidget>

namespace prop_arm::infra::ros {
class LimitsRegistry;
}  // namespace prop_arm::infra::ros

namespace prop_arm::models {
class TelemetryStore;
struct TelemetrySample;
enum class Domain : int;
}  // namespace prop_arm::models

namespace prop_arm::ui::widgets {
class AnglePlot;
class SpeedPlot;
class PwmPlot;
class DutyPlot;
}  // namespace prop_arm::ui::widgets

namespace prop_arm::ui::pages {

class DashboardPage final : public QWidget {
    Q_OBJECT

public:
    explicit DashboardPage(QWidget* parent = nullptr);

    void setTelemetryStore(prop_arm::models::TelemetryStore* store) noexcept;
    void setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept;

    void onTelemetry(const prop_arm::models::TelemetrySample& sample);
    void setStreamLive(prop_arm::models::Domain domain, bool live);

private:
    void buildUi_();
    void wireLimits_() noexcept;

private:
    prop_arm::models::TelemetryStore* store_{nullptr};
    const prop_arm::infra::ros::LimitsRegistry* limits_{nullptr};
    prop_arm::ui::widgets::AnglePlot* angle_plot_{nullptr};
    prop_arm::ui::widgets::SpeedPlot* speed_plot_{nullptr};
    prop_arm::ui::widgets::PwmPlot* pwm_plot_{nullptr};
    prop_arm::ui::widgets::DutyPlot* duty_plot_{nullptr};
};

}  // namespace prop_arm::ui::pages

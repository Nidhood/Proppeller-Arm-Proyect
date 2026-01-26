#pragma once

#include <QWidget>
#include <chrono>

namespace prop_arm::infra::ros {
class LimitsRegistry;
}

namespace prop_arm::models {
struct TelemetrySample;
enum class Domain : int;
}

namespace prop_arm::ui::widgets {
class ChartBase;
}

namespace prop_arm::ui::widgets {

class TrackingErrorPlot final : public QWidget {
    Q_OBJECT

public:
    explicit TrackingErrorPlot(QWidget* parent = nullptr);

    void setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setStreamLive(prop_arm::models::Domain domain, bool live);

    void pushSample(const prop_arm::models::TelemetrySample& sample);
    void clear();

private:
    static std::chrono::steady_clock::time_point sampleTime_(const prop_arm::models::TelemetrySample& s);

    void applyLimits_();

private:
    ChartBase* chart_{nullptr};
    const prop_arm::infra::ros::LimitsRegistry* limits_{nullptr};

    bool have_t0_{false};
    std::chrono::steady_clock::time_point t0_{};
};

}  // namespace prop_arm::ui::widgets

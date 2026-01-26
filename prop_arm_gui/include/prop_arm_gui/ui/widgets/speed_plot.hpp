#pragma once

#include <QWidget>
#include <chrono>

class QFrame;
class QVBoxLayout;

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

class SpeedPlot final : public QWidget {
    Q_OBJECT

public:
    explicit SpeedPlot(QWidget* parent = nullptr);

    void setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setStreamLive(prop_arm::models::Domain domain, bool live);

    void pushSample(const prop_arm::models::TelemetrySample& sample);
    void clear();

private:
    static std::chrono::steady_clock::time_point sampleTime_(const prop_arm::models::TelemetrySample& s);

    double tipSpeedMs_(double rad_s) const;
    static QWidget* makeChartFrame_(QWidget* child, const char* object_name);

private:
    QVBoxLayout* root_{nullptr};
    QFrame* frame_{nullptr};
    ChartBase* chart_{nullptr};

    const prop_arm::infra::ros::LimitsRegistry* limits_{nullptr};

    bool have_t0_{false};
    std::chrono::steady_clock::time_point t0_{};

    double kPropRadiusM_{0.125};  // 25 cm diameter -> 0.125 m radius
};

}  // namespace prop_arm::ui::widgets

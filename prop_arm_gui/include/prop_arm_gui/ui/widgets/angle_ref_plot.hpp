#pragma once

#include <QWidget>
#include <chrono>

namespace prop_arm::infra::ros {
class LimitsRegistry;
}

namespace prop_arm::models {
struct TelemetrySample;
enum class Domain;
}

namespace prop_arm::ui::widgets {
class ChartBase;
}

namespace prop_arm::ui::widgets {

class AngleRefPlot final : public QWidget {
    Q_OBJECT

public:
    explicit AngleRefPlot(QWidget* parent = nullptr);

    void setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setStreamLive(prop_arm::models::Domain domain, bool live);

    // Push ANY incoming telemetry (Real/Sim samples and optionally Ref samples).
    void pushSample(const prop_arm::models::TelemetrySample& sample);

    void clear();

private:
    void buildUi_();
    void applyLimits_();

    static std::chrono::steady_clock::time_point sampleTime_(const prop_arm::models::TelemetrySample& s);

    // Store latest reference (prefer Real ref; fallback to Sim ref if Real never arrived).
    void updateHeldRef_(prop_arm::models::Domain domain, double ref_deg);

    // Append held reference at current time to keep the ref curve continuous.
    void appendHeldRefAt_(double t_sec);

private:
    ChartBase* chart_{nullptr};
    const prop_arm::infra::ros::LimitsRegistry* limits_{nullptr};

    bool have_t0_{false};
    std::chrono::steady_clock::time_point t0_{};

    // Live tracking (ChartBase does not expose showSim()).
    bool live_real_{false};
    bool live_sim_{false};

    // Reference holding state for continuous visualization.
    bool have_real_ref_{false};
    bool have_ref_{false};
    double held_ref_deg_{0.0};
};

}  // namespace prop_arm::ui::widgets

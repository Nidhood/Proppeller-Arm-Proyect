#pragma once

#include <QWidget>

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"

QT_BEGIN_NAMESPACE
class QButtonGroup;
class QDoubleSpinBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QSlider;
class QSpinBox;
class QTimer;
class QGroupBox;
QT_END_NAMESPACE

namespace prop_arm::core::services {
class RecorderService;
}

namespace prop_arm::infra::ros {
class LimitsRegistry;
}

namespace prop_arm::ui::widgets {
class AngleRefPlot;
class ArmVisualizer;
class ChartFrame;
class TrackingErrorPlot;
}  // namespace prop_arm::ui::widgets

namespace prop_arm::ui::pages {

class ControlPanelPage final : public QWidget {
    Q_OBJECT

public:
    explicit ControlPanelPage(QWidget* parent = nullptr);

    void setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setRecorderService(prop_arm::core::services::RecorderService* recorder);

    void onTelemetry(const prop_arm::models::TelemetrySample& sample);
    void setStreamLive(prop_arm::models::Domain domain, bool live);

signals:
    void autoModeChanged(bool enabled);
    void stopRequested();
    void stabilizeRequested();

    void refAngleChanged(double rad);
    void pwmChanged(std::uint16_t pwm_us);

private slots:
    void onStopClicked_();
    void startStepTest_();
    void stopStepTest_();
    void onStepTimerTimeout_();

    void startRecording_();
    void stopRecording_();
    void refreshRecording_();
    void exportRecording_();

    void onRecordingUiTick_();

private:
    static double clampd_(double v, double lo, double hi) noexcept;
    static int clampi_(int v, int lo, int hi) noexcept;

    // UI building
    void buildUi_();
    QWidget* buildControlsWidget_();
    QGroupBox* buildModeGroup_();
    QGroupBox* buildManualGroup_();
    QGroupBox* buildStepGroup_();
    QGroupBox* buildRecordingGroup_();

    // Wiring + appearance
    void connectSignals_();
    void applyModeButtonTheme_();
    void setAutoModeUi_(bool auto_mode);
    void applyManualEnabled_();

    // Emission (throttled)
    void emitManualRefAngle_(double deg, bool force);
    void emitManualPwm_(int pwm_us, bool force);

    // Limits
    void wireLimits_();
    void applySafetyRanges_();

    // Recording UI/state
    void setRecordingUiIdle_();
    void setRecordingUiRecording_(double duration_s);
    void setRecordingUiStopped_();
    void setRecordingUiCompleted_(std::size_t points);
    void updateRecordingUiFromStats_();

    // Recorder callbacks (GUI thread via invokeMethod)
    void onRecordingProgress_(double remaining_s, std::size_t points);
    void onRecordingCompleted_(std::size_t points, double duration_s);

private:
    // Services
    prop_arm::core::services::RecorderService* recorder_service_{nullptr};
    const prop_arm::infra::ros::LimitsRegistry* limits_{nullptr};

    // Widgets
    prop_arm::ui::widgets::ChartFrame* arm_frame_{nullptr};
    prop_arm::ui::widgets::ArmVisualizer* arm_viz_{nullptr};

    prop_arm::ui::widgets::ChartFrame* controls_frame_{nullptr};
    prop_arm::ui::widgets::AngleRefPlot* angle_ref_plot_{nullptr};
    prop_arm::ui::widgets::TrackingErrorPlot* error_plot_{nullptr};

    // Mode
    QButtonGroup* mode_btn_group_{nullptr};
    QPushButton* manual_btn_{nullptr};
    QPushButton* auto_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
    QPushButton* stabilize_btn_{nullptr};

    // Manual controls
    QSlider* angle_slider_{nullptr};
    QDoubleSpinBox* angle_spinbox_{nullptr};

    QSlider* pwm_slider_{nullptr};
    QSpinBox* pwm_spinbox_{nullptr};

    // Step test
    QDoubleSpinBox* step_angle_low_{nullptr};
    QDoubleSpinBox* step_angle_high_{nullptr};
    QDoubleSpinBox* step_time_up_{nullptr};
    QDoubleSpinBox* step_time_down_{nullptr};
    QPushButton* start_step_btn_{nullptr};
    QPushButton* stop_step_btn_{nullptr};
    QTimer* step_timer_{nullptr};

    // Recording
    QDoubleSpinBox* recording_duration_{nullptr};
    QPushButton* start_recording_btn_{nullptr};
    QPushButton* stop_recording_btn_{nullptr};
    QLabel* recording_status_{nullptr};
    QProgressBar* recording_progress_{nullptr};
    QPushButton* refresh_btn_{nullptr};
    QPushButton* export_btn_{nullptr};
    QTimer* recording_ui_timer_{nullptr};

    // State
    bool auto_mode_{false};
    bool real_live_{false};
    bool sim_live_{false};

    qint64 last_emit_ms_angle_{0};
    qint64 last_emit_ms_pwm_{0};

    // Step timer parameters / state
    bool step_current_high_{false};
    int step_time_up_ms_{0};
    int step_time_down_ms_{0};

    // Recording state
    bool recording_active_{false};
    double recording_target_duration_{0.0};
    double recording_remaining_s_{0.0};
    std::size_t recording_points_{0};
    std::chrono::steady_clock::time_point recording_last_ui_tick_{};

    QTimer* stop_ramp_timer_{nullptr};
    bool stop_ramping_{false};
    bool stop_latched_{false};
    int stop_ramp_current_us_{0};
    int stop_ramp_target_us_{0};

    void requestStopRamp_();
    void onStopRampTick_();

    // UI sync helpers (no emission)
    void setAngleUiDeg_(double deg);
    void setPwmUiUs_(int pwm_us);

    // Derived enable logic
    void updateDerivedEnabled_();

    // Button styling
    void applyActionButtonsTheme_();

    // Stabilize support
    std::optional<double> pickLiveActualAngleRad_() const;

    // Cached last values (telemetry -> UI sync)
    double last_real_angle_rad_{0.0};
    double last_sim_angle_rad_{0.0};
    double last_ref_angle_rad_{0.0};
    int last_pwm_us_{0};
};

}  // namespace prop_arm::ui::pages

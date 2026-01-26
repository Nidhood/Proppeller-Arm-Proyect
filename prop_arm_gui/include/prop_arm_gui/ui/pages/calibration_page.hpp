#pragma once

#include <QWidget>
#include <QString>

#include "prop_arm_gui/core/models/calibration_types.hpp"
#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QStackedWidget;
class QPushButton;

namespace prop_arm::infra::ros {
class LimitsRegistry;
}

namespace prop_arm::services {
class CalibrationService;
}

namespace prop_arm::ui::widgets {
class AnglePlot;
class ArmVisualizer;
class ErrorPlot;
class Panel;
class ValueTile;
}

namespace prop_arm::ui::pages {

class CalibrationPage final : public QWidget {
    Q_OBJECT

public:
    explicit CalibrationPage(QWidget* parent = nullptr);

    void setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept;
    void setCalibrationService(prop_arm::services::CalibrationService* service) noexcept;

    void setStreamLive(prop_arm::models::Domain domain, bool live);
    void onTelemetry(const prop_arm::models::TelemetrySample& sample);

signals:
    void startCalibrationRequested(prop_arm::models::CalibrationConfig cfg);
    void stopCalibrationRequested();
    void applyParametersRequested();
    void resetCalibrationRequested();

private slots:
    void onStartClicked_();
    void onStopClicked_();
    void onApplyClicked_();
    void onResetClicked_();
    void onCalibrationTypeChanged_();

    void onStatusChanged_(prop_arm::models::CalibrationStatus status);
    void onProgressUpdated_(double progress01);
    void onCalibrationCompleted_(prop_arm::models::CalibrationResult result);
    void onMetricsUpdated_(prop_arm::models::CalibrationMetrics metrics);
    void onParametersChanged_();

private:
    void buildUi_();
    void wireLimits_();
    void wireCalibrationService_();

    void updateMetricsDisplay_();
    void updateParameterDisplay_();

    [[nodiscard]] prop_arm::models::CalibrationConfig buildConfig_() const;
    [[nodiscard]] QString statusToString_(prop_arm::models::CalibrationStatus status) const;

private:
    // External wiring
    const prop_arm::infra::ros::LimitsRegistry* limits_{nullptr};
    prop_arm::services::CalibrationService* calibration_service_{nullptr};

    // Layout: 1x2 (left calibration, right visuals)
    prop_arm::ui::widgets::Panel* calibration_panel_{nullptr};

    // Right side
    prop_arm::ui::widgets::Panel* arm_panel_{nullptr};
    prop_arm::ui::widgets::Panel* error_panel_{nullptr};

    // Visual widgets
    prop_arm::ui::widgets::ArmVisualizer* arm_viz_{nullptr};
    prop_arm::ui::widgets::AnglePlot* angle_plot_{nullptr};
    prop_arm::ui::widgets::ErrorPlot* error_plot_{nullptr};

    // Calibration sub-grid groups (inside calibration_panel_)
    QGroupBox* status_group_{nullptr};
    QGroupBox* config_group_{nullptr};
    QGroupBox* metrics_group_{nullptr};
    QGroupBox* params_group_{nullptr};

    // Status widgets
    QLabel* status_{nullptr};
    prop_arm::ui::widgets::ValueTile* progress_{nullptr};
    QPushButton* start_{nullptr};
    QPushButton* stop_{nullptr};
    QPushButton* apply_{nullptr};
    QPushButton* reset_{nullptr};

    // Config widgets
    QComboBox* calibration_type_{nullptr};
    QDoubleSpinBox* duration_{nullptr};
    QDoubleSpinBox* settling_time_{nullptr};
    QCheckBox* auto_apply_{nullptr};

    // Metrics tiles
    prop_arm::ui::widgets::ValueTile* rmse_tile_{nullptr};
    prop_arm::ui::widgets::ValueTile* max_error_tile_{nullptr};
    prop_arm::ui::widgets::ValueTile* mean_error_tile_{nullptr};
    prop_arm::ui::widgets::ValueTile* correlation_tile_{nullptr};

    // Parameters stacked
    QStackedWidget* params_stack_{nullptr};

    // Motor params
    QDoubleSpinBox* kw_spin_{nullptr};
    QDoubleSpinBox* tau_w_spin_{nullptr};
    QDoubleSpinBox* l_w_spin_{nullptr};
    QDoubleSpinBox* motor_scale_spin_{nullptr};

    // Physics params
    QDoubleSpinBox* mass_spin_{nullptr};
    QDoubleSpinBox* inertia_spin_{nullptr};
    QDoubleSpinBox* damping_spin_{nullptr};
    QDoubleSpinBox* friction_spin_{nullptr};

    // Telemetry cache to support absolute error behavior
    bool seen_real_{false};
    bool seen_sim_{false};
    double last_real_angle_{0.0};
    double last_sim_angle_{0.0};
};

}  // namespace prop_arm::ui::pages

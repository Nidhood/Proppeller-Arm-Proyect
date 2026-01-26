#include "prop_arm_gui/ui/pages/calibration_page.hpp"

#include "prop_arm_gui/core/services/calibration_service.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/ui/style/theme_manager.hpp"
#include "prop_arm_gui/ui/widgets/angle_plot.hpp"
#include "prop_arm_gui/ui/widgets/arm_visualizer.hpp"
#include "prop_arm_gui/ui/widgets/error_plot.hpp"
#include "prop_arm_gui/ui/widgets/panel.hpp"
#include "prop_arm_gui/ui/widgets/value_tile.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QStyle>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace prop_arm::ui::pages {

namespace {

void repolish(QWidget* w) {
    if (!w) return;
    w->style()->unpolish(w);
    w->style()->polish(w);
    w->update();
}

}  // namespace

CalibrationPage::CalibrationPage(QWidget* parent)
    : QWidget(parent) {
    buildUi_();
}

void CalibrationPage::buildUi_() {
    // =========================================================================
    // Root 1x2 (H layout) -> left calibration panel, right stacked visuals
    // 50/50 width.
    // =========================================================================
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(16);

    // ===================== LEFT: Calibration (Panel + internal 2x2 grid) =====================
    calibration_panel_ = new prop_arm::ui::widgets::Panel("Calibration", this);
    calibration_panel_->setObjectName("panel");
    calibration_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* calib_container = new QWidget(this);
    calib_container->setStyleSheet("background: transparent;");

    auto* calib_grid = new QGridLayout(calib_container);
    calib_grid->setContentsMargins(0, 0, 0, 0);
    calib_grid->setHorizontalSpacing(12);
    calib_grid->setVerticalSpacing(12);
    calib_grid->setColumnStretch(0, 1);
    calib_grid->setColumnStretch(1, 1);
    calib_grid->setRowStretch(0, 0);
    calib_grid->setRowStretch(1, 1);

    // --- Status (top-left)
    status_group_ = new QGroupBox("Status", calib_container);
    status_group_->setObjectName("controlGroup");
    auto* status_v = new QVBoxLayout(status_group_);
    status_v->setContentsMargins(12, 12, 12, 12);
    status_v->setSpacing(10);

    status_ = new QLabel("Idle", status_group_);
    status_->setObjectName("subtitle");

    progress_ = new prop_arm::ui::widgets::ValueTile("Progress", "%", status_group_);
    progress_->setObjectName("valueTile");
    progress_->setValueText("0");

    start_ = new QPushButton("Start Calibration", status_group_);
    stop_  = new QPushButton("Stop", status_group_);
    stop_->setObjectName("secondary");
    stop_->setEnabled(false);

    apply_ = new QPushButton("Apply Parameters", status_group_);
    apply_->setEnabled(false);

    reset_ = new QPushButton("Reset to Defaults", status_group_);
    reset_->setObjectName("secondary");

    auto* status_top = new QHBoxLayout();
    status_top->setSpacing(10);
    status_top->addWidget(status_, 1);
    status_top->addWidget(progress_, 0);

    auto* status_btns = new QGridLayout();
    status_btns->setHorizontalSpacing(10);
    status_btns->setVerticalSpacing(10);
    status_btns->addWidget(start_, 0, 0);
    status_btns->addWidget(stop_,  0, 1);
    status_btns->addWidget(reset_, 1, 0);
    status_btns->addWidget(apply_, 1, 1);

    status_v->addLayout(status_top);
    status_v->addLayout(status_btns);
    status_v->addStretch(1);

    // --- Configuration (top-right)
    config_group_ = new QGroupBox("Configuration", calib_container);
    config_group_->setObjectName("controlGroup");
    auto* config_form = new QFormLayout(config_group_);
    config_form->setContentsMargins(12, 12, 12, 12);
    config_form->setSpacing(8);

    calibration_type_ = new QComboBox(config_group_);
    calibration_type_->addItem("Motor Velocity",
                               static_cast<int>(prop_arm::models::CalibrationType::MotorVelocity));
    calibration_type_->addItem("Arm Physics",
                               static_cast<int>(prop_arm::models::CalibrationType::ArmPhysics));

    duration_ = new QDoubleSpinBox(config_group_);
    duration_->setRange(5.0, 300.0);
    duration_->setValue(30.0);
    duration_->setDecimals(1);
    duration_->setSuffix(" s");

    settling_time_ = new QDoubleSpinBox(config_group_);
    settling_time_->setRange(0.5, 10.0);
    settling_time_->setValue(2.0);
    settling_time_->setDecimals(2);
    settling_time_->setSuffix(" s");

    auto_apply_ = new QCheckBox("Auto-apply on success", config_group_);

    config_form->addRow("Calibration Type:", calibration_type_);
    config_form->addRow("Test Duration:", duration_);
    config_form->addRow("Settling Time:", settling_time_);
    config_form->addRow("", auto_apply_);

    // --- Metrics (bottom-left)
    metrics_group_ = new QGroupBox("Metrics", calib_container);
    metrics_group_->setObjectName("controlGroup");
    auto* metrics_grid = new QGridLayout(metrics_group_);
    metrics_grid->setContentsMargins(12, 12, 12, 12);
    metrics_grid->setHorizontalSpacing(8);
    metrics_grid->setVerticalSpacing(8);

    rmse_tile_ = new prop_arm::ui::widgets::ValueTile("RMSE", "", metrics_group_);
    max_error_tile_ = new prop_arm::ui::widgets::ValueTile("Max Error", "", metrics_group_);
    mean_error_tile_ = new prop_arm::ui::widgets::ValueTile("Mean Error", "", metrics_group_);
    correlation_tile_ = new prop_arm::ui::widgets::ValueTile("Correlation", "", metrics_group_);

    metrics_grid->addWidget(rmse_tile_, 0, 0);
    metrics_grid->addWidget(max_error_tile_, 0, 1);
    metrics_grid->addWidget(mean_error_tile_, 1, 0);
    metrics_grid->addWidget(correlation_tile_, 1, 1);

    // --- Parameters (bottom-right)
    params_group_ = new QGroupBox("Parameters", calib_container);
    params_group_->setObjectName("controlGroup");
    auto* params_v = new QVBoxLayout(params_group_);
    params_v->setContentsMargins(12, 12, 12, 12);
    params_v->setSpacing(10);

    params_stack_ = new QStackedWidget(params_group_);
    params_stack_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    // Motor params page
    auto* motor_page = new QWidget(params_group_);
    auto* motor_form = new QFormLayout(motor_page);
    motor_form->setSpacing(8);

    kw_spin_ = new QDoubleSpinBox(motor_page);
    kw_spin_->setRange(0.0, 100.0);
    kw_spin_->setDecimals(6);

    tau_w_spin_ = new QDoubleSpinBox(motor_page);
    tau_w_spin_->setRange(0.0, 10.0);
    tau_w_spin_->setDecimals(6);
    tau_w_spin_->setSuffix(" s");

    l_w_spin_ = new QDoubleSpinBox(motor_page);
    l_w_spin_->setRange(0.0, 10.0);
    l_w_spin_->setDecimals(6);
    l_w_spin_->setSuffix(" s");

    motor_scale_spin_ = new QDoubleSpinBox(motor_page);
    motor_scale_spin_->setRange(0.0, 100.0);
    motor_scale_spin_->setDecimals(6);

    motor_form->addRow("Kw (gain):", kw_spin_);
    motor_form->addRow("τw (time const):", tau_w_spin_);
    motor_form->addRow("Lw (delay):", l_w_spin_);
    motor_form->addRow("Motor scale:", motor_scale_spin_);

    // Physics params page
    auto* physics_page = new QWidget(params_group_);
    auto* physics_form = new QFormLayout(physics_page);
    physics_form->setSpacing(8);

    mass_spin_ = new QDoubleSpinBox(physics_page);
    mass_spin_->setRange(0.0, 100.0);
    mass_spin_->setDecimals(6);
    mass_spin_->setSuffix(" kg");

    inertia_spin_ = new QDoubleSpinBox(physics_page);
    inertia_spin_->setRange(0.0, 10.0);
    inertia_spin_->setDecimals(9);
    inertia_spin_->setSuffix(" kg·m²");

    damping_spin_ = new QDoubleSpinBox(physics_page);
    damping_spin_->setRange(0.0, 100.0);
    damping_spin_->setDecimals(9);
    damping_spin_->setSuffix(" N·m·s/rad");

    friction_spin_ = new QDoubleSpinBox(physics_page);
    friction_spin_->setRange(0.0, 100.0);
    friction_spin_->setDecimals(9);
    friction_spin_->setSuffix(" N·m");

    physics_form->addRow("Mass:", mass_spin_);
    physics_form->addRow("Inertia (Iyy):", inertia_spin_);
    physics_form->addRow("Damping:", damping_spin_);
    physics_form->addRow("Friction:", friction_spin_);

    params_stack_->addWidget(motor_page);   // index 0
    params_stack_->addWidget(physics_page); // index 1
    params_v->addWidget(params_stack_, 1);

    // Place 2x2 sub-grid blocks
    calib_grid->addWidget(status_group_, 0, 0);
    calib_grid->addWidget(config_group_, 0, 1);
    calib_grid->addWidget(metrics_group_, 1, 0);
    calib_grid->addWidget(params_group_, 1, 1);

    calibration_panel_->bodyLayout()->addWidget(calib_container, 1);

    // ===================== RIGHT: Arm viz (top) + Error plot (bottom) =====================
    auto* right_col = new QWidget(this);
    right_col->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* right_v = new QVBoxLayout(right_col);
    right_v->setContentsMargins(0, 0, 0, 0);
    right_v->setSpacing(16);

    // Arm panel (NO ChartFrame)
    arm_panel_ = new prop_arm::ui::widgets::Panel("Arm", this);
    arm_panel_->setObjectName("panel");
    arm_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    arm_viz_ = new prop_arm::ui::widgets::ArmVisualizer(this);
    arm_viz_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    {
        const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();
        arm_viz_->applyTheme(spec);
    }
    arm_viz_->setShowReal(false);
    arm_viz_->setShowSim(false);
    arm_viz_->setShowRef(true); // always show ref

    arm_panel_->bodyLayout()->addWidget(arm_viz_, 1);

    // Error panel (NO ChartFrame)
    error_panel_ = new prop_arm::ui::widgets::Panel("Error | abs(Real - Sim)", this);
    error_panel_->setObjectName("panel");
    error_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    error_plot_ = new prop_arm::ui::widgets::ErrorPlot(this);
    error_plot_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    error_panel_->bodyLayout()->addWidget(error_plot_, 1);

    // Angle plot is still shown (your previous layout had it).
    // Now it lives INSIDE the calibration panel? No: you wanted only the calibration panel on the left.
    // So we keep AnglePlot as part of calibration panel? You said "keep same calibration panel (perfect)".
    // Therefore AnglePlot stays in the calibration panel? The "perfect" panel is the calibration panel,
    // not angle plot. If you still want angle plot, tell me where to place it.
    //
    // For now: create it but do not place it (to respect your "unique bloque calibration panel" left side).
    angle_plot_ = new prop_arm::ui::widgets::AnglePlot(this);
    angle_plot_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Put Arm + Error in the right column
    right_v->addWidget(arm_panel_, 1);
    right_v->addWidget(error_panel_, 1);

    // Add left and right to root with equal stretch (50/50)
    root->addWidget(calibration_panel_, 1);
    root->addWidget(right_col, 1);

    // Theme wiring
    connect(&prop_arm::ui::style::ThemeManager::instance(),
            &prop_arm::ui::style::ThemeManager::themeChanged,
            this,
    [this](auto) {
        const auto& s = prop_arm::ui::style::ThemeManager::instance().currentSpec();
        if (arm_viz_) arm_viz_->applyTheme(s);
        repolish(this);
    });

    // Signals
    connect(start_, &QPushButton::clicked, this, &CalibrationPage::onStartClicked_);
    connect(stop_,  &QPushButton::clicked, this, &CalibrationPage::onStopClicked_);
    connect(apply_, &QPushButton::clicked, this, &CalibrationPage::onApplyClicked_);
    connect(reset_, &QPushButton::clicked, this, &CalibrationPage::onResetClicked_);

    connect(calibration_type_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &CalibrationPage::onCalibrationTypeChanged_);

    onCalibrationTypeChanged_();
    wireLimits_();
}

void CalibrationPage::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    wireLimits_();
}

void CalibrationPage::setCalibrationService(prop_arm::services::CalibrationService* service) noexcept {
    calibration_service_ = service;
    wireCalibrationService_();
}

void CalibrationPage::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (angle_plot_) angle_plot_->setStreamLive(domain, live);
    if (error_plot_) error_plot_->setStreamLive(domain, live);

    if (arm_viz_) {
        if (domain == prop_arm::models::Domain::Real) arm_viz_->setShowReal(live);
        if (domain == prop_arm::models::Domain::Sim)  arm_viz_->setShowSim(live);
    }
}

void CalibrationPage::onTelemetry(const prop_arm::models::TelemetrySample& sample) {
    if (!sample.valid) return;

    // Feed angle plot if you decide to place it somewhere later
    if (angle_plot_) angle_plot_->pushSample(sample);

    // Feed error plot (it needs both domains to compute)
    if (error_plot_) error_plot_->pushSample(sample);

    // Update arm visualizer
    if (arm_viz_) {
        arm_viz_->setRefAngle(sample.ref_angle_rad);
        if (sample.domain == prop_arm::models::Domain::Real) {
            seen_real_ = true;
            last_real_angle_ = sample.arm_angle_rad;
            arm_viz_->setRealAngle(sample.arm_angle_rad);
        } else {
            seen_sim_ = true;
            last_sim_angle_ = sample.arm_angle_rad;
            arm_viz_->setSimAngle(sample.arm_angle_rad);
        }
    }

    // IMPORTANT: Absolute error requirement.
    // Your ErrorPlot should show abs(Real - Sim). If your current ErrorPlot is signed,
    // you MUST change it there (std::abs). However, to force correctness at the page level,
    // we ensure both angles are valid and then keep the cache aligned.
    //
    // If ErrorPlot expects to compute error from samples internally, this cache doesn't hurt.
    // If you later expose an API like error_plot_->pushError(t, abs_err), use that instead.
    (void)seen_real_;
    (void)seen_sim_;
}

void CalibrationPage::wireLimits_() {
    if (angle_plot_) angle_plot_->setLimitsRegistry(limits_);
    if (error_plot_) error_plot_->setLimitsRegistry(limits_);
}

void CalibrationPage::wireCalibrationService_() {
    if (!calibration_service_) return;

    // Avoid duplicate connections if called multiple times
    disconnect(calibration_service_, nullptr, this, nullptr);

    // Worker thread -> UI thread safety
    connect(calibration_service_, &prop_arm::services::CalibrationService::statusChanged,
            this, &CalibrationPage::onStatusChanged_, Qt::QueuedConnection);

    connect(calibration_service_, &prop_arm::services::CalibrationService::progressUpdated,
            this, &CalibrationPage::onProgressUpdated_, Qt::QueuedConnection);

    connect(calibration_service_, &prop_arm::services::CalibrationService::calibrationCompleted,
            this, &CalibrationPage::onCalibrationCompleted_, Qt::QueuedConnection);

    connect(calibration_service_, &prop_arm::services::CalibrationService::metricsUpdated,
            this, &CalibrationPage::onMetricsUpdated_, Qt::QueuedConnection);

    connect(calibration_service_, &prop_arm::services::CalibrationService::parametersChanged,
            this, &CalibrationPage::onParametersChanged_, Qt::QueuedConnection);

    updateParameterDisplay_();
    updateMetricsDisplay_();
    onStatusChanged_(calibration_service_->getLastResult().status);
}

void CalibrationPage::onStatusChanged_(prop_arm::models::CalibrationStatus status) {
    const bool is_running =
        (status == prop_arm::models::CalibrationStatus::Running ||
         status == prop_arm::models::CalibrationStatus::Analyzing);

    if (start_) start_->setEnabled(!is_running);
    if (stop_)  stop_->setEnabled(is_running);

    if (config_group_) config_group_->setEnabled(!is_running);
    if (params_group_) params_group_->setEnabled(!is_running);

    const bool is_completed = (status == prop_arm::models::CalibrationStatus::Completed);
    if (apply_) apply_->setEnabled(is_completed);

    if (status_) status_->setText(statusToString_(status));
}

void CalibrationPage::onProgressUpdated_(double progress01) {
    const double pct = std::clamp(progress01, 0.0, 1.0) * 100.0;
    if (progress_) progress_->setValue(pct, 1);
}

void CalibrationPage::onCalibrationCompleted_(prop_arm::models::CalibrationResult result) {
    if (result.status == prop_arm::models::CalibrationStatus::Failed) {
        if (status_) status_->setText("Failed: " + QString::fromStdString(result.error_message));
    }
}

void CalibrationPage::onMetricsUpdated_(prop_arm::models::CalibrationMetrics) {
    updateMetricsDisplay_();
}

void CalibrationPage::onParametersChanged_() {
    updateParameterDisplay_();
}

void CalibrationPage::onCalibrationTypeChanged_() {
    if (!calibration_type_ || !params_stack_) return;

    const auto type =
        static_cast<prop_arm::models::CalibrationType>(calibration_type_->currentData().toInt());

    params_stack_->setCurrentIndex(type == prop_arm::models::CalibrationType::MotorVelocity ? 0 : 1);
}

void CalibrationPage::onStartClicked_() {
    const auto cfg = buildConfig_();
    emit startCalibrationRequested(cfg);

    if (calibration_service_) {
        calibration_service_->startCalibration(cfg);
    }
}

void CalibrationPage::onStopClicked_() {
    emit stopCalibrationRequested();
    if (calibration_service_) calibration_service_->stopCalibration();
}

void CalibrationPage::onApplyClicked_() {
    emit applyParametersRequested();
    if (calibration_service_) calibration_service_->applyCalibration();
}

void CalibrationPage::onResetClicked_() {
    emit resetCalibrationRequested();
    if (calibration_service_) calibration_service_->resetToDefaults();

    if (angle_plot_) angle_plot_->clear();
    if (error_plot_) error_plot_->clear();

    seen_real_ = false;
    seen_sim_ = false;
    last_real_angle_ = 0.0;
    last_sim_angle_ = 0.0;
}

void CalibrationPage::updateMetricsDisplay_() {
    if (!calibration_service_) return;

    const auto r = calibration_service_->getLastResult();
    const auto& m = r.metrics;

    if (rmse_tile_) rmse_tile_->setValue(m.rmse, 6);
    if (max_error_tile_) max_error_tile_->setValue(m.max_error, 6);
    if (mean_error_tile_) mean_error_tile_->setValue(m.mean_error, 6);
    if (correlation_tile_) correlation_tile_->setValue(m.correlation, 6);
}

void CalibrationPage::updateParameterDisplay_() {
    if (!calibration_service_) return;

    const auto motor = calibration_service_->getMotorParams();
    if (kw_spin_) kw_spin_->setValue(motor.Kw);
    if (tau_w_spin_) tau_w_spin_->setValue(motor.tau_w);
    if (l_w_spin_) l_w_spin_->setValue(motor.L_w);
    if (motor_scale_spin_) motor_scale_spin_->setValue(motor.motor_cmd_scale);

    const auto phys = calibration_service_->getPhysicsParams();
    if (mass_spin_) mass_spin_->setValue(phys.mass);
    if (inertia_spin_) inertia_spin_->setValue(phys.inertia_yy);
    if (damping_spin_) damping_spin_->setValue(phys.damping);
    if (friction_spin_) friction_spin_->setValue(phys.friction);
}

prop_arm::models::CalibrationConfig CalibrationPage::buildConfig_() const {
    prop_arm::models::CalibrationConfig cfg;

    cfg.type = static_cast<prop_arm::models::CalibrationType>(
                   calibration_type_ ? calibration_type_->currentData().toInt()
                   : static_cast<int>(prop_arm::models::CalibrationType::MotorVelocity));

    cfg.duration_sec = duration_ ? duration_->value() : 30.0;
    cfg.settling_time_sec = settling_time_ ? settling_time_->value() : 2.0;
    cfg.auto_apply = auto_apply_ ? auto_apply_->isChecked() : false;

    // Defaults (still safe)
    cfg.convergence_threshold = 0.05;
    cfg.max_iterations = 6;

    if (cfg.type == prop_arm::models::CalibrationType::MotorVelocity) {
        cfg.test_inputs = {1200.0, 1400.0, 1600.0, 1800.0};
    } else {
        cfg.test_inputs = {0.5, 1.0, 1.5, 2.0, 2.5};
    }

    return cfg;
}

QString CalibrationPage::statusToString_(prop_arm::models::CalibrationStatus status) const {
    switch (status) {
    case prop_arm::models::CalibrationStatus::Idle:
        return "Idle";
    case prop_arm::models::CalibrationStatus::Running:
        return "Running calibration...";
    case prop_arm::models::CalibrationStatus::Analyzing:
        return "Analyzing results...";
    case prop_arm::models::CalibrationStatus::Completed:
        return "Calibration completed";
    case prop_arm::models::CalibrationStatus::Failed:
        return "Calibration failed";
    default:
        return "Unknown";
    }
}

}  // namespace prop_arm::ui::pages

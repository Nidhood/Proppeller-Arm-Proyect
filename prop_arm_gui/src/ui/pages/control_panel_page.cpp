#include "prop_arm_gui/ui/pages/control_panel_page.hpp"

#include "prop_arm_gui/core/models/csv_export_options.hpp"
#include "prop_arm_gui/core/services/recorder_service.hpp"
#include "prop_arm_gui/infra/ros/limits_registry.hpp"
#include "prop_arm_gui/ui/style/theme_manager.hpp"
#include "prop_arm_gui/ui/widgets/angle_ref_plot.hpp"
#include "prop_arm_gui/ui/widgets/arm_visualizer.hpp"
#include "prop_arm_gui/ui/widgets/chart_frame.hpp"
#include "prop_arm_gui/ui/widgets/export_preview_dialog.hpp"
#include "prop_arm_gui/ui/widgets/tracking_error_plot.hpp"

#include <QButtonGroup>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QStyle>
#include <algorithm>
#include <cmath>

namespace prop_arm::ui::pages {

namespace {

constexpr int kEmitThrottleMs = 45;
constexpr int kStopRampIntervalMs = 20;     // PWM ramp cadence
constexpr int kStopRampStepUs = 8;          // PWM decrement per tick (smooth)
constexpr int kUiAngleDecimals = 2;
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;

static void applySliderAccent(QSlider* s, const QColor& accent) {
    if (!s) return;

    const QString css = QString(R"(
        QSlider::groove:horizontal {
            height: 6px;
            border-radius: 3px;
            background: rgba(255,255,255,0.12);
        }
        QSlider::sub-page:horizontal {
            border-radius: 3px;
            background: %1;
        }
        QSlider::add-page:horizontal {
            border-radius: 3px;
            background: rgba(255,255,255,0.12);
        }
        QSlider::handle:horizontal {
            width: 14px;
            height: 14px;
            margin: -5px 0;
            border-radius: 7px;
            background: %1;
            border: 1px solid rgba(0,0,0,0.25);
        }
        QSlider::handle:horizontal:hover {
            background: %1;
        }
    )").arg(accent.name());

    s->setStyleSheet(css);
}

static void polishWidget(QWidget* w) {
    if (!w) return;
    w->style()->unpolish(w);
    w->style()->polish(w);
    w->update();
}

}  // namespace

// ------------------------ Ctor / helpers -------------------------

ControlPanelPage::ControlPanelPage(QWidget* parent)
    : QWidget(parent) {
    buildUi_();

    recording_ui_timer_ = new QTimer(this);
    recording_ui_timer_->setInterval(125);
    connect(recording_ui_timer_, &QTimer::timeout, this, &ControlPanelPage::onRecordingUiTick_);

    stop_ramp_timer_ = new QTimer(this);
    stop_ramp_timer_->setInterval(kStopRampIntervalMs);
    connect(stop_ramp_timer_, &QTimer::timeout, this, [this]() {
        onStopRampTick_();
    });

    setRecordingUiIdle_();
}

double ControlPanelPage::clampd_(double v, double lo, double hi) noexcept {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int ControlPanelPage::clampi_(int v, int lo, int hi) noexcept {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ------------------------ UI: build -------------------------

void ControlPanelPage::buildUi_() {
    auto* root = new QGridLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setHorizontalSpacing(16);
    root->setVerticalSpacing(16);
    root->setColumnStretch(0, 1);
    root->setColumnStretch(1, 1);
    root->setRowStretch(0, 1);
    root->setRowStretch(1, 1);

    arm_viz_ = new prop_arm::ui::widgets::ArmVisualizer(this);
    arm_viz_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    {
        const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();
        arm_viz_->applyTheme(spec);
    }
    arm_viz_->setShowReal(false);
    arm_viz_->setShowSim(false);
    arm_viz_->setShowRef(false);

    arm_frame_ = new prop_arm::ui::widgets::ChartFrame(this);
    arm_frame_->setObjectName("chartFrame");
    arm_frame_->setChartWidget(arm_viz_);
    arm_frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    root->addWidget(arm_frame_, 0, 0);

    auto* controls_widget = buildControlsWidget_();
    controls_frame_ = new prop_arm::ui::widgets::ChartFrame(this);
    controls_frame_->setObjectName("chartFrame");
    controls_frame_->setChartWidget(controls_widget);
    controls_frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    root->addWidget(controls_frame_, 1, 0);

    angle_ref_plot_ = new prop_arm::ui::widgets::AngleRefPlot(this);
    angle_ref_plot_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    error_plot_ = new prop_arm::ui::widgets::TrackingErrorPlot(this);
    error_plot_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    root->addWidget(angle_ref_plot_, 0, 1);
    root->addWidget(error_plot_, 1, 1);

    {
        const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();
        applySliderAccent(angle_slider_, spec.accent);
        applySliderAccent(pwm_slider_, spec.accent2);
    }

    connect(&prop_arm::ui::style::ThemeManager::instance(),
            &prop_arm::ui::style::ThemeManager::themeChanged,
            this,
    [this](auto) {
        const auto& s = prop_arm::ui::style::ThemeManager::instance().currentSpec();
        applySliderAccent(angle_slider_, s.accent);
        applySliderAccent(pwm_slider_, s.accent2);
        applyModeButtonTheme_();
        applyActionButtonsTheme_();
        if (arm_viz_) arm_viz_->applyTheme(s);
    });

    setAutoModeUi_(false);
    applyModeButtonTheme_();
    applyActionButtonsTheme_();
    connectSignals_();
    wireLimits_();
    updateDerivedEnabled_();
}

QWidget* ControlPanelPage::buildControlsWidget_() {
    auto* w = new QWidget(this);
    w->setStyleSheet("background: transparent;");

    auto* grid = new QGridLayout(w);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setHorizontalSpacing(12);
    grid->setVerticalSpacing(12);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    grid->addWidget(buildModeGroup_(), 0, 0);
    grid->addWidget(buildManualGroup_(), 0, 1);
    grid->addWidget(buildStepGroup_(), 1, 0);
    grid->addWidget(buildRecordingGroup_(), 1, 1);

    return w;
}

QGroupBox* ControlPanelPage::buildModeGroup_() {
    auto* mode_group = new QGroupBox("Control Mode");
    mode_group->setObjectName("controlGroup");
    auto* mode_layout = new QVBoxLayout(mode_group);
    mode_layout->setSpacing(10);

    auto* mode_row = new QHBoxLayout();
    mode_row->setSpacing(8);

    manual_btn_ = new QPushButton("MANUAL");
    manual_btn_->setObjectName("modeSegBtn");
    manual_btn_->setCheckable(true);
    manual_btn_->setChecked(true);
    manual_btn_->setMinimumHeight(36);

    auto_btn_ = new QPushButton("AUTO");
    auto_btn_->setObjectName("modeSegBtn");
    auto_btn_->setCheckable(true);
    auto_btn_->setChecked(false);
    auto_btn_->setMinimumHeight(36);

    mode_btn_group_ = new QButtonGroup(this);
    mode_btn_group_->setExclusive(true);
    mode_btn_group_->addButton(manual_btn_, 0);
    mode_btn_group_->addButton(auto_btn_, 1);
    manual_btn_->setProperty("segPos", "left");
    auto_btn_->setProperty("segPos", "right");

    mode_row->addWidget(manual_btn_, 1);
    mode_row->addWidget(auto_btn_, 1);
    mode_layout->addLayout(mode_row);

    auto* mode_buttons = new QHBoxLayout();
    mode_buttons->setSpacing(8);

    stop_btn_ = new QPushButton("STOP");
    stop_btn_->setObjectName("stopButton");
    stop_btn_->setMinimumHeight(40);

    stabilize_btn_ = new QPushButton("STABILIZE");
    stabilize_btn_->setObjectName("stabilizeButton");
    stabilize_btn_->setMinimumHeight(40);

    mode_buttons->addWidget(stop_btn_);
    mode_buttons->addWidget(stabilize_btn_);
    mode_layout->addLayout(mode_buttons);

    return mode_group;
}

QGroupBox* ControlPanelPage::buildManualGroup_() {
    auto* manual_group = new QGroupBox("Manual Commands");
    manual_group->setObjectName("controlGroup");
    auto* manual_layout = new QVBoxLayout(manual_group);
    manual_layout->setSpacing(10);

    // Reference Angle
    auto* angle_group = new QGroupBox("Reference Angle");
    angle_group->setObjectName("controlSubGroup");
    auto* angle_layout = new QGridLayout(angle_group);
    angle_layout->setSpacing(8);
    angle_layout->setColumnStretch(1, 1);

    angle_slider_ = new QSlider(Qt::Horizontal);
    angle_slider_->setObjectName("controlSlider");
    angle_slider_->setRange(0, 90);
    angle_slider_->setValue(0);
    angle_slider_->setTickPosition(QSlider::NoTicks);

    angle_spinbox_ = new QDoubleSpinBox();
    angle_spinbox_->setObjectName("controlSpinbox");
    angle_spinbox_->setRange(0.0, 90.0);
    angle_spinbox_->setValue(0.0);
    angle_spinbox_->setSuffix(" °");
    angle_spinbox_->setDecimals(kUiAngleDecimals);
    angle_spinbox_->setMaximumWidth(120);
    angle_spinbox_->setMinimumHeight(32);

    auto* angle_label = new QLabel("Angle:");
    angle_label->setObjectName("controlLabel");

    angle_layout->addWidget(angle_label, 0, 0);
    angle_layout->addWidget(angle_slider_, 0, 1);
    angle_layout->addWidget(angle_spinbox_, 0, 2);

    // PWM Input
    auto* pwm_group = new QGroupBox("PWM Input");
    pwm_group->setObjectName("controlSubGroup");
    auto* pwm_layout = new QGridLayout(pwm_group);
    pwm_layout->setSpacing(8);
    pwm_layout->setColumnStretch(1, 1);

    pwm_slider_ = new QSlider(Qt::Horizontal);
    pwm_slider_->setObjectName("controlSlider");
    pwm_slider_->setRange(1000, 2000);
    pwm_slider_->setValue(1000);
    pwm_slider_->setTickPosition(QSlider::NoTicks);

    pwm_spinbox_ = new QSpinBox();
    pwm_spinbox_->setObjectName("controlSpinbox");
    pwm_spinbox_->setRange(1000, 2000);
    pwm_spinbox_->setValue(1000);
    pwm_spinbox_->setSuffix(" µs");
    pwm_spinbox_->setMaximumWidth(120);
    pwm_spinbox_->setMinimumHeight(32);

    auto* pwm_label = new QLabel("PWM:");
    pwm_label->setObjectName("controlLabel");

    pwm_layout->addWidget(pwm_label, 0, 0);
    pwm_layout->addWidget(pwm_slider_, 0, 1);
    pwm_layout->addWidget(pwm_spinbox_, 0, 2);

    manual_layout->addWidget(angle_group);
    manual_layout->addWidget(pwm_group);

    return manual_group;
}

QGroupBox* ControlPanelPage::buildStepGroup_() {
    auto* step_group = new QGroupBox("Step Test");
    step_group->setObjectName("controlGroup");
    auto* step_layout = new QGridLayout(step_group);
    step_layout->setSpacing(8);

    step_angle_low_ = new QDoubleSpinBox();
    step_angle_low_->setObjectName("controlSpinbox");
    step_angle_low_->setRange(0.0, 90.0);
    step_angle_low_->setValue(0.0);
    step_angle_low_->setSuffix(" °");
    step_angle_low_->setDecimals(1);

    step_angle_high_ = new QDoubleSpinBox();
    step_angle_high_->setObjectName("controlSpinbox");
    step_angle_high_->setRange(0.0, 90.0);
    step_angle_high_->setValue(45.0);
    step_angle_high_->setSuffix(" °");
    step_angle_high_->setDecimals(1);

    step_time_up_ = new QDoubleSpinBox();
    step_time_up_->setObjectName("controlSpinbox");
    step_time_up_->setRange(0.1, 60.0);
    step_time_up_->setValue(5.0);
    step_time_up_->setSuffix(" s");
    step_time_up_->setDecimals(1);

    step_time_down_ = new QDoubleSpinBox();
    step_time_down_->setObjectName("controlSpinbox");
    step_time_down_->setRange(0.1, 60.0);
    step_time_down_->setValue(5.0);
    step_time_down_->setSuffix(" s");
    step_time_down_->setDecimals(1);

    auto* low_label = new QLabel("Low:");
    low_label->setObjectName("controlLabel");
    auto* high_label = new QLabel("High:");
    high_label->setObjectName("controlLabel");
    auto* time_up_label = new QLabel("Time Up:");
    time_up_label->setObjectName("controlLabel");
    auto* time_down_label = new QLabel("Down:");
    time_down_label->setObjectName("controlLabel");

    step_layout->addWidget(low_label, 0, 0);
    step_layout->addWidget(step_angle_low_, 0, 1);
    step_layout->addWidget(high_label, 0, 2);
    step_layout->addWidget(step_angle_high_, 0, 3);
    step_layout->addWidget(time_up_label, 1, 0);
    step_layout->addWidget(step_time_up_, 1, 1);
    step_layout->addWidget(time_down_label, 1, 2);
    step_layout->addWidget(step_time_down_, 1, 3);

    start_step_btn_ = new QPushButton("START STEP");
    start_step_btn_->setObjectName("actionButton");
    start_step_btn_->setMinimumHeight(36);

    stop_step_btn_ = new QPushButton("STOP STEP");
    stop_step_btn_->setObjectName("dangerButton");
    stop_step_btn_->setMinimumHeight(36);
    stop_step_btn_->setEnabled(false);

    step_layout->addWidget(start_step_btn_, 2, 0, 1, 2);
    step_layout->addWidget(stop_step_btn_, 2, 2, 1, 2);

    return step_group;
}

QGroupBox* ControlPanelPage::buildRecordingGroup_() {
    auto* rec_group = new QGroupBox("Recording");
    rec_group->setObjectName("controlGroup");
    auto* rec_layout = new QGridLayout(rec_group);
    rec_layout->setSpacing(8);

    recording_duration_ = new QDoubleSpinBox();
    recording_duration_->setObjectName("controlSpinbox");
    recording_duration_->setRange(5.0, 600.0);
    recording_duration_->setValue(120.0);
    recording_duration_->setSuffix(" s");
    recording_duration_->setDecimals(1);

    start_recording_btn_ = new QPushButton("START REC");
    start_recording_btn_->setObjectName("actionButton");
    start_recording_btn_->setMinimumHeight(36);

    stop_recording_btn_ = new QPushButton("STOP REC");
    stop_recording_btn_->setObjectName("warningButton");
    stop_recording_btn_->setMinimumHeight(36);
    stop_recording_btn_->setEnabled(false);

    recording_status_ = new QLabel("Not Recording");
    recording_status_->setObjectName("statusLabel");
    recording_status_->setAlignment(Qt::AlignCenter);

    recording_progress_ = new QProgressBar();
    recording_progress_->setObjectName("recordingProgress");
    recording_progress_->setRange(0, 100);
    recording_progress_->setValue(0);
    recording_progress_->setFormat("%v s");
    recording_progress_->setVisible(false);
    recording_progress_->setMinimumHeight(28);

    refresh_btn_ = new QPushButton("REFRESH");
    refresh_btn_->setObjectName("secondaryButton");
    refresh_btn_->setMinimumHeight(36);

    export_btn_ = new QPushButton("EXPORT");
    export_btn_->setObjectName("accentButton");
    export_btn_->setMinimumHeight(36);
    export_btn_->setEnabled(false);

    auto* dur_label = new QLabel("Duration:");
    dur_label->setObjectName("controlLabel");

    rec_layout->addWidget(dur_label, 0, 0);
    rec_layout->addWidget(recording_duration_, 0, 1);
    rec_layout->addWidget(start_recording_btn_, 0, 2);
    rec_layout->addWidget(stop_recording_btn_, 0, 3);
    rec_layout->addWidget(recording_status_, 1, 0, 1, 4);
    rec_layout->addWidget(recording_progress_, 2, 0, 1, 4);
    rec_layout->addWidget(refresh_btn_, 3, 0, 1, 2);
    rec_layout->addWidget(export_btn_, 3, 2, 1, 2);

    return rec_group;
}

// ------------------------ Styling -------------------------

void ControlPanelPage::applyModeButtonTheme_() {
    const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();

    const QString segStyle = QString(R"(
        QPushButton#modeSegBtn {
            background: %1;
            color: %2;
            border: 1px solid %3;
            padding: 7px 16px;
            font-weight: 700;
            letter-spacing: 0.5px;
        }
        QPushButton#modeSegBtn[segPos="left"] { border-radius: 6px; }
        QPushButton#modeSegBtn[segPos="right"] { border-radius: 6px; }

        QPushButton#modeSegBtn:hover:!checked {
            background: %4;
            color: %5;
        }

        QPushButton#modeSegBtn:checked {
            background: %6;
            color: %7;
            border-color: %6;
        }

        QPushButton#modeSegBtn:disabled {
            background: rgba(255,255,255,0.06);
            color: rgba(255,255,255,0.28);
            border-color: rgba(255,255,255,0.10);
        }
    )")
                             .arg(spec.panel.lighter(108).name())
                             .arg(spec.text_muted.name())
                             .arg(spec.text_muted.lighter(130).name())
                             .arg(spec.panel.lighter(120).name())
                             .arg(spec.text.name())
                             .arg(spec.accent.name())
                             .arg(spec.bg.name());

    if (manual_btn_) manual_btn_->setStyleSheet(segStyle);
    if (auto_btn_) auto_btn_->setStyleSheet(segStyle);
}

void ControlPanelPage::applyActionButtonsTheme_() {
    const auto& s = prop_arm::ui::style::ThemeManager::instance().currentSpec();

    const QString common = QString(R"(
        QPushButton {
            border-radius: 8px;
            padding: 10px 18px;
            font-weight: 700;
            letter-spacing: 0.8px;
            border: 2px solid transparent;
        }
        QPushButton:disabled {
            background-color: transparent !important;
            border: 2px solid %1 !important;
            color: %1 !important;
            opacity: 0.3;
        }
    )").arg(s.text_muted.name());

    const QString stopCss = common + QString(R"(
        QPushButton#stopButton {
            background-color: rgba(255, 60, 60, 0.15);
            color: %1;
            border-color: rgba(255, 60, 60, 0.4);
        }
        QPushButton#stopButton:hover:enabled {
            background-color: #FF3C3C;
            border-color: #FF3C3C;
            color: %2;
        }
    )").arg(s.text.name()).arg(s.bg.name());

    const QString stabCss = common + QString(R"(
        QPushButton#stabilizeButton {
            background-color: rgba(80, 200, 255, 0.15);
            color: %1;
            border-color: rgba(80, 200, 255, 0.4);
        }
        QPushButton#stabilizeButton:hover:enabled {
            background-color: #50C8FF;
            border-color: #50C8FF;
            color: %2;
        }
    )").arg(s.text.name()).arg(s.bg.name());

    const QString actionCss = common + QString(R"(
        QPushButton#actionButton {
            background-color: rgba(120, 255, 120, 0.12);
            color: %1;
            border-color: rgba(120, 255, 120, 0.35);
        }
        QPushButton#actionButton:hover:enabled {
            background-color: #78FF78;
            border-color: #78FF78;
            color: %2;
        }
    )").arg(s.text.name()).arg(s.bg.name());

    const QString warnCss = common + QString(R"(
        QPushButton#warningButton {
            background-color: rgba(255, 200, 60, 0.12);
            color: %1;
            border-color: rgba(255, 200, 60, 0.35);
        }
        QPushButton#warningButton:hover:enabled {
            background-color: #FFC83C;
            border-color: #FFC83C;
            color: %2;
        }
    )").arg(s.text.name()).arg(s.bg.name());

    const QString dangerCss = common + QString(R"(
        QPushButton#dangerButton {
            background-color: rgba(255, 80, 120, 0.12);
            color: %1;
            border-color: rgba(255, 80, 120, 0.35);
        }
        QPushButton#dangerButton:hover:enabled {
            background-color: #FF5078;
            border-color: #FF5078;
            color: %2;
        }
    )").arg(s.text.name()).arg(s.bg.name());

    const QString secondaryCss = common + QString(R"(
        QPushButton#secondaryButton {
            background-color: transparent;
            color: %1;
            border-color: %1;
        }
        QPushButton#secondaryButton:hover:enabled {
            background-color: %2;
            border-color: %2;
            color: %3;
        }
    )").arg(s.text_muted.name()).arg(s.accent.name()).arg(s.bg.name());

    const QString accentCss = common + QString(R"(
        QPushButton#accentButton {
            background-color: %1;
            color: %2;
            border-color: %1;
        }
        QPushButton#accentButton:hover:enabled {
            background-color: %3;
            border-color: %3;
            color: %2;
        }
        QPushButton#accentButton:disabled {
            background-color: transparent !important;
            border: 2px solid %4 !important;
            color: %4 !important;
            opacity: 0.3;
        }
    )")
                              .arg(s.accent.name())
                              .arg(s.bg.name())
                              .arg(s.accent.lighter(115).name())
                              .arg(s.text_muted.name());

    if (stop_btn_) stop_btn_->setStyleSheet(stopCss);
    if (stabilize_btn_) stabilize_btn_->setStyleSheet(stabCss);

    if (start_step_btn_) start_step_btn_->setStyleSheet(actionCss);
    if (start_recording_btn_) start_recording_btn_->setStyleSheet(actionCss);

    if (stop_step_btn_) stop_step_btn_->setStyleSheet(dangerCss);
    if (stop_recording_btn_) stop_recording_btn_->setStyleSheet(warnCss);

    if (refresh_btn_) refresh_btn_->setStyleSheet(secondaryCss);
    if (export_btn_) export_btn_->setStyleSheet(accentCss);

    // Force style refresh on enabled state changes later
    polishWidget(stop_btn_);
    polishWidget(stabilize_btn_);
    polishWidget(start_step_btn_);
    polishWidget(stop_step_btn_);
    polishWidget(start_recording_btn_);
    polishWidget(stop_recording_btn_);
    polishWidget(refresh_btn_);
    polishWidget(export_btn_);
}

// ------------------------ Signal wiring -------------------------

void ControlPanelPage::connectSignals_() {
    connect(mode_btn_group_, &QButtonGroup::idClicked, this, [this](int id) {
        const bool auto_mode = (id == 1);
        setAutoModeUi_(auto_mode);
        emit autoModeChanged(auto_mode);
    });

    connect(stop_btn_, &QPushButton::clicked, this, &ControlPanelPage::onStopClicked_);

    connect(stabilize_btn_, &QPushButton::clicked, this, [this]() {
        // Stabilize behavior:
        // 1) Pick actual angle from active stream (Real preferred)
        // 2) Set reference to that angle
        // 3) Emit stabilizeRequested (backend can switch to stabilize mode)
        const std::optional<double> actual = pickLiveActualAngleRad_();
        if (actual.has_value()) {
            stop_latched_ = false;
            stop_ramping_ = false;
            if (stop_ramp_timer_ && stop_ramp_timer_->isActive()) stop_ramp_timer_->stop();

            const double deg = clampd_((*actual) * kRad2Deg,
                                       angle_spinbox_->minimum(),
                                       angle_spinbox_->maximum());
            setAngleUiDeg_(deg);
            emit refAngleChanged(deg * kDeg2Rad);
        }
        emit stabilizeRequested();
    });

    // Angle (manual)
    connect(angle_slider_, &QSlider::valueChanged, this, [this](int value) {
        // Any user action clears stop latch
        stop_latched_ = false;

        {
            QSignalBlocker b(*angle_spinbox_);
            angle_spinbox_->setValue(static_cast<double>(value));
        }
        emitManualRefAngle_(static_cast<double>(value), false);
    });

    connect(angle_slider_, &QSlider::sliderReleased, this, [this]() {
        stop_latched_ = false;
        emitManualRefAngle_(angle_spinbox_->value(), true);
    });

    connect(angle_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    [this](double value) {
        stop_latched_ = false;

        const int v = clampi_(static_cast<int>(std::lround(value)),
                              angle_slider_->minimum(),
                              angle_slider_->maximum());
        {
            QSignalBlocker b(*angle_slider_);
            angle_slider_->setValue(v);
        }
        emitManualRefAngle_(value, false);
    });

    // PWM (manual)
    connect(pwm_slider_, &QSlider::valueChanged, this, [this](int value) {
        stop_latched_ = false;

        {
            QSignalBlocker b(*pwm_spinbox_);
            pwm_spinbox_->setValue(value);
        }
        emitManualPwm_(value, false);
    });

    connect(pwm_slider_, &QSlider::sliderReleased, this, [this]() {
        stop_latched_ = false;
        emitManualPwm_(pwm_spinbox_->value(), true);
    });

    connect(pwm_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        stop_latched_ = false;

        const int v = clampi_(value, pwm_slider_->minimum(), pwm_slider_->maximum());
        {
            QSignalBlocker b(*pwm_slider_);
            pwm_slider_->setValue(v);
        }
        emitManualPwm_(v, false);
    });

    // Step
    connect(start_step_btn_, &QPushButton::clicked, this, &ControlPanelPage::startStepTest_);
    connect(stop_step_btn_, &QPushButton::clicked, this, &ControlPanelPage::stopStepTest_);

    // Recording
    connect(start_recording_btn_, &QPushButton::clicked, this, &ControlPanelPage::startRecording_);
    connect(stop_recording_btn_, &QPushButton::clicked, this, &ControlPanelPage::stopRecording_);
    connect(refresh_btn_, &QPushButton::clicked, this, &ControlPanelPage::refreshRecording_);
    connect(export_btn_, &QPushButton::clicked, this, &ControlPanelPage::exportRecording_);
}

// ------------------------ Mode + enable logic -------------------------

void ControlPanelPage::setAutoModeUi_(bool auto_mode) {
    auto_mode_ = auto_mode;
    applyManualEnabled_();
    updateDerivedEnabled_();
}

void ControlPanelPage::applyManualEnabled_() {
    const bool manual_enabled = !auto_mode_;
    angle_slider_->setEnabled(manual_enabled);
    angle_spinbox_->setEnabled(manual_enabled);
    pwm_slider_->setEnabled(manual_enabled);
    pwm_spinbox_->setEnabled(manual_enabled);
}

void ControlPanelPage::updateDerivedEnabled_() {
    // Step test should visually show unavailable when auto is not enabled
    if (start_step_btn_) start_step_btn_->setEnabled(auto_mode_);
    if (stop_step_btn_ && !auto_mode_) stop_step_btn_->setEnabled(false);

    // Export should be enabled only if data exists (also reflects dim style via :disabled)
    if (export_btn_) {
        const bool has_points = (recorder_service_ && !recorder_service_->recorded().empty());
        export_btn_->setEnabled(has_points);
    }

    polishWidget(start_step_btn_);
    polishWidget(stop_step_btn_);
    polishWidget(export_btn_);
}

// ------------------------ Emit (manual) -------------------------

void ControlPanelPage::emitManualRefAngle_(double deg, bool force) {
    // In auto mode, manual emissions are blocked (but telemetry will still update UI).
    if (auto_mode_) return;

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (!force && (now_ms - last_emit_ms_angle_) < kEmitThrottleMs) return;
    last_emit_ms_angle_ = now_ms;

    const double deg_clamped = clampd_(deg, angle_spinbox_->minimum(), angle_spinbox_->maximum());
    last_ref_angle_rad_ = deg_clamped * kDeg2Rad;
    emit refAngleChanged(last_ref_angle_rad_);
}

void ControlPanelPage::emitManualPwm_(int pwm_us, bool force) {
    if (auto_mode_) return;

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (!force && (now_ms - last_emit_ms_pwm_) < kEmitThrottleMs) return;
    last_emit_ms_pwm_ = now_ms;

    const int v = clampi_(pwm_us, pwm_spinbox_->minimum(), pwm_spinbox_->maximum());
    last_pwm_us_ = v;
    emit pwmChanged(static_cast<std::uint16_t>(v));
}

// ------------------------ STOP behavior (smooth ramp + hold) -------------------------

void ControlPanelPage::onStopClicked_() {
    if (step_timer_ && step_timer_->isActive()) stopStepTest_();
    if (recording_active_) stopRecording_();

    // Smooth ramp PWM to minimum and hold until user changes controls
    requestStopRamp_();

    // Keep higher-level stop semantics (controller can switch mode/state)
    emit stopRequested();
}

void ControlPanelPage::requestStopRamp_() {
    if (!pwm_spinbox_) return;

    stop_latched_ = true;
    stop_ramping_ = true;

    // Target min PWM
    stop_ramp_target_us_ = pwm_spinbox_->minimum();

    // Start from current UI PWM (or last known)
    stop_ramp_current_us_ = pwm_spinbox_->value();
    stop_ramp_current_us_ = clampi_(stop_ramp_current_us_, pwm_spinbox_->minimum(), pwm_spinbox_->maximum());

    if (!stop_ramp_timer_->isActive()) stop_ramp_timer_->start();
}

void ControlPanelPage::onStopRampTick_() {
    if (!stop_ramping_) {
        if (stop_ramp_timer_->isActive()) stop_ramp_timer_->stop();
        return;
    }

    const int min_pwm = stop_ramp_target_us_;
    int cur = stop_ramp_current_us_;

    if (cur <= min_pwm) {
        // Done: hold min visually and as last command
        cur = min_pwm;
        stop_ramp_current_us_ = cur;
        stop_ramping_ = false;

        setPwmUiUs_(cur);

        last_pwm_us_ = cur;
        emit pwmChanged(static_cast<std::uint16_t>(cur));

        if (stop_ramp_timer_->isActive()) stop_ramp_timer_->stop();
        return;
    }

    // Ramp down
    cur = std::max(min_pwm, cur - kStopRampStepUs);
    stop_ramp_current_us_ = cur;

    // Update UI (even in auto) + emit PWM every tick for smooth decay
    setPwmUiUs_(cur);

    last_pwm_us_ = cur;
    emit pwmChanged(static_cast<std::uint16_t>(cur));
}

// ------------------------ STABILIZE support -------------------------

std::optional<double> ControlPanelPage::pickLiveActualAngleRad_() const {
    if (real_live_) return last_real_angle_rad_;
    if (sim_live_) return last_sim_angle_rad_;
    return std::nullopt;
}

// ------------------------ UI setters (safe, no emission) -------------------------

void ControlPanelPage::setAngleUiDeg_(double deg) {
    const double d = clampd_(deg, angle_spinbox_->minimum(), angle_spinbox_->maximum());
    const int slider_v = clampi_(static_cast<int>(std::lround(d)),
                                 angle_slider_->minimum(),
                                 angle_slider_->maximum());

    {
        QSignalBlocker b1(*angle_spinbox_);
        QSignalBlocker b2(*angle_slider_);
        angle_spinbox_->setValue(d);
        angle_slider_->setValue(slider_v);
    }

    // Track what UI shows (useful when switching modes)
    last_ref_angle_rad_ = d * kDeg2Rad;
}

void ControlPanelPage::setPwmUiUs_(int pwm_us) {
    const int v = clampi_(pwm_us, pwm_spinbox_->minimum(), pwm_spinbox_->maximum());
    {
        QSignalBlocker b1(*pwm_spinbox_);
        QSignalBlocker b2(*pwm_slider_);
        pwm_spinbox_->setValue(v);
        pwm_slider_->setValue(v);
    }
    last_pwm_us_ = v;
}

// ------------------------ Step test -------------------------

void ControlPanelPage::startStepTest_() {
    if (step_timer_ && step_timer_->isActive()) return;

    if (!auto_mode_) {
        // With proper disabling this should rarely happen, but keep safety.
        QMessageBox::warning(this, "Step Test", "Auto mode must be enabled to run step test.");
        return;
    }

    const double low = step_angle_low_->value();
    const double high = step_angle_high_->value();
    const double time_up = step_time_up_->value();
    const double time_down = step_time_down_->value();

    if (high <= low) {
        QMessageBox::warning(this, "Step Test", "High angle must be greater than low angle.");
        return;
    }

    if (!step_timer_) {
        step_timer_ = new QTimer(this);
        step_timer_->setSingleShot(true);
        connect(step_timer_, &QTimer::timeout, this, &ControlPanelPage::onStepTimerTimeout_);
    }

    start_step_btn_->setEnabled(false);
    stop_step_btn_->setEnabled(true);
    polishWidget(start_step_btn_);
    polishWidget(stop_step_btn_);

    step_time_up_ms_ = static_cast<int>(std::lround(time_up * 1000.0));
    step_time_down_ms_ = static_cast<int>(std::lround(time_down * 1000.0));

    step_current_high_ = true;

    // Set UI + emit
    setAngleUiDeg_(high);
    emit refAngleChanged(high * kDeg2Rad);

    step_timer_->start(step_time_up_ms_);
}

void ControlPanelPage::stopStepTest_() {
    if (!step_timer_) return;

    step_timer_->stop();
    start_step_btn_->setEnabled(auto_mode_);
    stop_step_btn_->setEnabled(false);
    polishWidget(start_step_btn_);
    polishWidget(stop_step_btn_);
}

void ControlPanelPage::onStepTimerTimeout_() {
    if (!step_timer_) return;

    const double low = step_angle_low_->value();
    const double high = step_angle_high_->value();

    if (step_current_high_) {
        step_current_high_ = false;
        setAngleUiDeg_(low);
        emit refAngleChanged(low * kDeg2Rad);
        step_timer_->start(step_time_down_ms_);
    } else {
        step_current_high_ = true;
        setAngleUiDeg_(high);
        emit refAngleChanged(high * kDeg2Rad);
        step_timer_->start(step_time_up_ms_);
    }
}

// ------------------------ Recording -------------------------

void ControlPanelPage::setRecorderService(prop_arm::core::services::RecorderService* recorder) {
    recorder_service_ = recorder;

    if (!recorder_service_) {
        updateDerivedEnabled_();
        return;
    }

    recorder_service_->setOnProgress([this](double remaining_s, std::size_t points) {
        QMetaObject::invokeMethod(this,
        [this, remaining_s, points]() {
            onRecordingProgress_(remaining_s, points);
        },
        Qt::QueuedConnection);
    });

    recorder_service_->setOnCompleted([this](std::size_t points, double duration_s) {
        QMetaObject::invokeMethod(this,
        [this, points, duration_s]() {
            onRecordingCompleted_(points, duration_s);
        },
        Qt::QueuedConnection);
    });

    updateRecordingUiFromStats_();
    updateDerivedEnabled_();
}

void ControlPanelPage::startRecording_() {
    if (recording_active_) return;

    if (!recorder_service_) {
        QMessageBox::warning(this, "Recording", "Recorder service not available.");
        return;
    }

    const double duration = recording_duration_->value();
    recording_active_ = true;
    recording_target_duration_ = duration;
    recording_remaining_s_ = duration;
    recording_points_ = 0;

    setRecordingUiRecording_(duration);

    recorder_service_->start(duration);

    recording_last_ui_tick_ = std::chrono::steady_clock::now();
    if (!recording_ui_timer_->isActive()) recording_ui_timer_->start();
}

void ControlPanelPage::stopRecording_() {
    if (!recording_active_) return;

    if (recorder_service_) recorder_service_->stop();

    recording_active_ = false;
    if (recording_ui_timer_ && recording_ui_timer_->isActive()) recording_ui_timer_->stop();

    setRecordingUiStopped_();
    updateDerivedEnabled_();
}

void ControlPanelPage::refreshRecording_() {
    if (recording_active_) stopRecording_();

    if (recorder_service_) recorder_service_->clear();
    setRecordingUiIdle_();
    updateDerivedEnabled_();
}

void ControlPanelPage::exportRecording_() {
    if (!recorder_service_) {
        QMessageBox::warning(this, "Export", "Recorder service not available.");
        return;
    }

    const auto recorded = recorder_service_->recorded();
    if (recorded.empty()) {
        QMessageBox::warning(this, "Export", "No recorded data available to export.");
        updateDerivedEnabled_();
        return;
    }

    QVector<prop_arm::models::TelemetrySample> samples;
    samples.reserve(static_cast<int>(recorded.size()));
    for (const auto& s : recorded) samples.push_back(s);

    const QString default_filename =
        QStringLiteral("proparm_recording_%1.csv")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    prop_arm::models::CsvExportOptions default_opts;
    default_opts.include_header_comments = true;
    default_opts.decimals = 6;
    default_opts.columns = {
        "Domain",
        "t_rel_s",
        "arm_angle_rad",
        "motor_speed_rad_s",
        "pwm_us",
        "ref_angle_rad",
        "valid"
    };

    prop_arm::ui::widgets::ExportPreviewDialog dlg(this);
    dlg.setWindowTitle("Export Recording");
    dlg.setSamples(samples);
    dlg.setDefaultFilename(default_filename);
    dlg.setDefaultOptions(default_opts);

    dlg.exec();
}

void ControlPanelPage::onRecordingUiTick_() {
    if (!recorder_service_) return;

    updateRecordingUiFromStats_();

    const auto st = recorder_service_->stats();
    if (recording_active_ && st.state == prop_arm::core::logging::RecordingState::Completed) {
        onRecordingCompleted_(st.points_total, st.duration_s);
    }

    updateDerivedEnabled_();
}

void ControlPanelPage::onRecordingProgress_(double remaining_s, std::size_t points) {
    if (!recording_active_) return;

    recording_remaining_s_ = std::max(0.0, remaining_s);
    recording_points_ = points;

    const double elapsed = std::max(0.0, recording_target_duration_ - recording_remaining_s_);
    recording_progress_->setValue(static_cast<int>(std::lround(elapsed)));
    recording_status_->setText(QString("Recording... %1 pts").arg(points));
    updateDerivedEnabled_();
}

void ControlPanelPage::onRecordingCompleted_(std::size_t points, double /*duration_s*/) {
    if (recording_ui_timer_ && recording_ui_timer_->isActive()) recording_ui_timer_->stop();

    recording_active_ = false;
    recording_points_ = points;

    setRecordingUiCompleted_(points);
    updateDerivedEnabled_();
}

void ControlPanelPage::setRecordingUiIdle_() {
    recording_active_ = false;
    recording_target_duration_ = 0.0;
    recording_remaining_s_ = 0.0;
    recording_points_ = 0;

    recording_status_->setText("Not Recording");
    recording_progress_->setVisible(false);
    recording_progress_->setValue(0);

    start_recording_btn_->setEnabled(true);
    stop_recording_btn_->setEnabled(false);
    recording_duration_->setEnabled(true);

    export_btn_->setEnabled(false);
}

void ControlPanelPage::setRecordingUiRecording_(double duration_s) {
    recording_status_->setText("Recording...");
    recording_progress_->setVisible(true);
    recording_progress_->setRange(0, static_cast<int>(std::ceil(duration_s)));
    recording_progress_->setValue(0);

    start_recording_btn_->setEnabled(false);
    stop_recording_btn_->setEnabled(true);
    recording_duration_->setEnabled(false);

    refresh_btn_->setEnabled(false);
    export_btn_->setEnabled(false);
}

void ControlPanelPage::setRecordingUiStopped_() {
    recording_status_->setText("Recording stopped");
    recording_progress_->setVisible(false);

    start_recording_btn_->setEnabled(true);
    stop_recording_btn_->setEnabled(false);
    recording_duration_->setEnabled(true);

    refresh_btn_->setEnabled(true);
}

void ControlPanelPage::setRecordingUiCompleted_(std::size_t points) {
    recording_status_->setText(QString("Complete: %1 pts").arg(points));
    recording_progress_->setVisible(false);

    start_recording_btn_->setEnabled(true);
    stop_recording_btn_->setEnabled(false);
    recording_duration_->setEnabled(true);

    refresh_btn_->setEnabled(true);
    export_btn_->setEnabled(points > 0);

    if (points > 0) {
        QMessageBox::information(
            this,
            "Recording Complete",
            QString("Successfully recorded %1 data points.\n\nClick 'EXPORT' to save to CSV.")
            .arg(points));
    }
}

void ControlPanelPage::updateRecordingUiFromStats_() {
    if (!recorder_service_) return;

    const auto st = recorder_service_->stats();
    if (st.state == prop_arm::core::logging::RecordingState::Recording) {
        if (!recording_active_) {
            recording_active_ = true;
            recording_target_duration_ = st.duration_s;
            setRecordingUiRecording_(st.duration_s);
        }
        recording_points_ = st.points_total;
        recording_remaining_s_ = st.remaining_s;

        const double elapsed = std::max(0.0, st.elapsed_s);
        recording_progress_->setValue(static_cast<int>(std::lround(elapsed)));
        recording_status_->setText(QString("Recording... %1 pts").arg(st.points_total));
    }
}

// ------------------------ Limits -------------------------

void ControlPanelPage::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) noexcept {
    limits_ = limits;
    wireLimits_();
}

void ControlPanelPage::wireLimits_() {
    applySafetyRanges_();
    if (angle_ref_plot_) angle_ref_plot_->setLimitsRegistry(limits_);
    if (error_plot_) error_plot_->setLimitsRegistry(limits_);
}

void ControlPanelPage::applySafetyRanges_() {
    (void)limits_;
}

// ------------------------ Telemetry + live flags -------------------------

void ControlPanelPage::onTelemetry(const prop_arm::models::TelemetrySample& sample) {
    if (!sample.valid) return;

    // Update cached actuals (used by Stabilize + UI sync)
    if (sample.domain == prop_arm::models::Domain::Real) {
        last_real_angle_rad_ = sample.arm_angle_rad;
    } else if (sample.domain == prop_arm::models::Domain::Sim) {
        last_sim_angle_rad_ = sample.arm_angle_rad;
    }

    // Always push to plots
    if (angle_ref_plot_) angle_ref_plot_->pushSample(sample);
    if (error_plot_) error_plot_->pushSample(sample);

    // Visualizer updates
    if (arm_viz_) {
        arm_viz_->setRefAngle(sample.ref_angle_rad);

        if (sample.domain == prop_arm::models::Domain::Real) {
            arm_viz_->setRealAngle(sample.arm_angle_rad);
        } else if (sample.domain == prop_arm::models::Domain::Sim) {
            arm_viz_->setSimAngle(sample.arm_angle_rad);
        }
    }

    // --- UI synchronization from telemetry (senior behavior) ---
    // 1) If stop latch is active, do NOT overwrite the UI with telemetry.
    //    (User pressed STOP and expects minimum PWM to remain.)
    if (stop_latched_) {
        return;
    }

    // 2) Track ref angle updates (e.g., auto controller changing it)
    //    Only update UI if we are in AUTO (or if user is not actively dragging).
    if (auto_mode_) {
        const double ref_deg = clampd_(sample.ref_angle_rad * kRad2Deg,
                                       angle_spinbox_->minimum(),
                                       angle_spinbox_->maximum());
        setAngleUiDeg_(ref_deg);
    }

    // 3) Track PWM always (so MANUAL shows last auto PWM when returning)
    //    PWM is an int in your sample; clamp to UI range.
    const int pwm = clampi_(static_cast<int>(sample.pwm_us),
                            pwm_spinbox_->minimum(),
                            pwm_spinbox_->maximum());

    if (auto_mode_) {
        setPwmUiUs_(pwm);
    }

    // Cache last known
    last_ref_angle_rad_ = sample.ref_angle_rad;
    last_pwm_us_ = pwm;

    updateDerivedEnabled_();
}

void ControlPanelPage::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (angle_ref_plot_) angle_ref_plot_->setStreamLive(domain, live);
    if (error_plot_) error_plot_->setStreamLive(domain, live);

    if (domain == prop_arm::models::Domain::Real) real_live_ = live;
    if (domain == prop_arm::models::Domain::Sim)  sim_live_  = live;

    if (arm_viz_) {
        arm_viz_->setShowReal(real_live_);
        arm_viz_->setShowSim(sim_live_);
        arm_viz_->setShowRef(real_live_ || sim_live_);
    }
}

}  // namespace prop_arm::ui::pages

#pragma once

#include <QMainWindow>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QTabWidget>
#include <QTimer>
#include <QProgressBar>
#include <QFrame>
#include <QFont>
#include <QPalette>
#include <QScreen>
#include <QGuiApplication>
#include <QMetaObject>
#include <QThread>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QCheckBox>
#include <memory>
#include <future>

#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include "prop_arm_gui/data_exporter.hpp"
#include "prop_arm_gui/chart_base.hpp"

class AerospaceDataVisualizer;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<PropArmGuiNode> node, QWidget *parent = nullptr);
    ~MainWindow() = default;

private slots:
    void updateDisplays();
    void onAngleSliderChanged(int value);
    void onPWMSliderChanged(int value);
    void onAutoModeToggled(bool checked);
    void onStopClicked();
    void onStabilizeClicked();
    void onRefreshClicked();
    void onExportDataClicked();
    void onStartRecordingClicked();
    void onStopRecordingClicked();
    void onRecordingStarted(double duration);
    void onRecordingProgress(double remaining_time, size_t point_count);
    void onRecordingCompleted(size_t point_count, double duration);
    void onStartStepTestClicked();
    void onStopStepTestClicked();

private:
    void setupUI();
    void setupStyles();
    void setupUbuntuScreenGeometry();
    void setupControlTab();
    void setupControlPanel();
    void setupControlCharts();
    void createStatusBar();

    std::shared_ptr<PropArmGuiNode> ros_node_;
    QTimer *update_timer_;

    QWidget *central_widget_;
    QTabWidget *tab_widget_;

    QWidget *control_widget_;
    AerospaceDataVisualizer *data_visualizer_;

    QGroupBox *control_group_;

    QSlider *angle_slider_;
    QSlider *pwm_slider_;
    QDoubleSpinBox *angle_spinbox_;
    QSpinBox *pwm_spinbox_;
    QCheckBox *auto_mode_checkbox_;
    QPushButton *stop_btn_;
    QPushButton *stabilize_btn_;
    QPushButton *refresh_btn_;
    QPushButton *export_btn_;

    QDoubleSpinBox *step_angle_low_spinbox_;
    QDoubleSpinBox *step_angle_high_spinbox_;
    QDoubleSpinBox *step_time_up_spinbox_;
    QDoubleSpinBox *step_time_down_spinbox_;
    QPushButton *start_step_test_btn_;
    QPushButton *stop_step_test_btn_;

    QDoubleSpinBox *recording_duration_spinbox_;
    QPushButton *start_recording_btn_;
    QPushButton *stop_recording_btn_;
    QLabel *recording_status_label_;
    QProgressBar *recording_progress_bar_;

    // NUEVAS GRÁFICAS PARA LA VENTANA DE CONTROL
    ChartBase *angle_vs_reference_chart_;
    ChartBase *error_chart_;
    ChartBase *motor_velocity_chart_;

    QLabel *connection_status_;
    QLabel *control_mode_;
    QLabel *system_status_;

    const QString PRIMARY_COLOR = "#1a365d";
    const QString SECONDARY_COLOR = "#2563eb";
    const QString SUCCESS_COLOR = "#00ff88";
    const QString WARNING_COLOR = "#ff8c00";
    const QString DANGER_COLOR = "#ff3366";
    const QString BACKGROUND_COLOR = "#080a0f";
    const QString CARD_COLOR = "#1e293b";
    const QString TEXT_COLOR = "#e2e8f0";
    const QString ACCENT_COLOR = "#00ccff";
};
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
#include <memory>
#include <future>


#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include "prop_arm_gui/data_exporter.hpp"

// Forward declaration
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
    void onVelocitySliderChanged(int value);
    void onStopClicked();
    void onStabilizeClicked();
    void onRefreshClicked();
    void onExportDataClicked();
    
    // Recording slots
    void onStartRecordingClicked();
    void onStopRecordingClicked();
    void onRecordingStarted(double duration);
    void onRecordingProgress(double remaining_time, size_t point_count);
    void onRecordingCompleted(size_t point_count, double duration);

private:
    void setupUI();
    void setupStyles();
    void setupUbuntuScreenGeometry();
    void setupControlTab();
    void setupControlPanel();
    void setupMonitoringPanel();
    void createStatusBar();

    // Core components
    std::shared_ptr<PropArmGuiNode> ros_node_;
    QTimer *update_timer_;

    // Main UI structure
    QWidget *central_widget_;
    QTabWidget *tab_widget_;

    // Tab widgets
    AerospaceDataVisualizer *data_visualizer_;
    QWidget *control_widget_;

    // Control panel components
    QGroupBox *control_group_;
    QSlider *angle_slider_;
    QSlider *velocity_slider_;
    QDoubleSpinBox *angle_spinbox_;
    QDoubleSpinBox *velocity_spinbox_;
    QPushButton *stop_btn_;
    QPushButton *stabilize_btn_;
    QPushButton *refresh_btn_;
    QPushButton *export_btn_;
    
    // Recording components
    QDoubleSpinBox *recording_duration_spinbox_;
    QPushButton *start_recording_btn_;
    QPushButton *stop_recording_btn_;
    QLabel *recording_status_label_;
    QProgressBar *recording_progress_bar_;

    // Monitoring panel components
    QGroupBox *monitor_group_;
    QLabel *arm_angle_value_;
    QLabel *motor_speed_value_;
    QLabel *pwm_input_value_;
    QLabel *duty_cycle_value_;
    QLabel *error_value_;
    QLabel *motor_cmd_value_;
    QProgressBar *angle_progress_;
    QProgressBar *velocity_progress_;

    // Status indicators
    QLabel *connection_status_;
    QLabel *control_mode_;
    QLabel *system_status_;

    // Aerospace color scheme
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
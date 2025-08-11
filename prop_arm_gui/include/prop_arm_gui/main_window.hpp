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
#include <QTimer>
#include <QProgressBar>
#include <QFrame>
#include <QFont>
#include <QPalette>
#include <memory>

#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include "prop_arm_gui/real_time_plot.hpp"
#include "prop_arm_gui/aerospace_dashboard.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<PropArmGuiNode> node, QWidget *parent = nullptr);
    ~MainWindow() = default;

private slots:
    void updateDisplays();
    void onAngleSliderChanged(int value);
    void onForceSliderChanged(int value);
    void onVelocitySliderChanged(int value);
    void onStopClicked();
    void onStabilizeClicked();
    void onRefreshClicked();

private:
    void setupUI();
    void setupStyles();
    void setupControlPanel();
    void setupMonitoringPanel();
    void setupPlotsPanel();
    void createStatusBar();

    // Core components
    std::shared_ptr<PropArmGuiNode> ros_node_;
    QTimer *update_timer_;

    // Main layouts
    QWidget *central_widget_;
    QGridLayout *main_layout_;

    // Control panel
    QGroupBox *control_group_;
    QSlider *angle_slider_;
    QSlider *force_slider_;
    QSlider *velocity_slider_;
    QDoubleSpinBox *angle_spinbox_;
    QDoubleSpinBox *force_spinbox_;
    QDoubleSpinBox *velocity_spinbox_;
    QPushButton *stop_btn_;
    QPushButton *stabilize_btn_;
    QPushButton *refresh_btn_;

    // Monitoring panel
    QGroupBox *monitor_group_;
    QLabel *arm_angle_value_;
    QLabel *motor_speed_value_;
    QLabel *v_emf_value_;
    QLabel *delta_v_emf_value_;
    QLabel *error_value_;
    QLabel *motor_cmd_value_;
    QProgressBar *thrust_progress_;
    QProgressBar *angle_progress_;

    // Aerospace dashboard
    AerospaceDashboard *dashboard_;

    // Real-time plots
    QGroupBox *plots_group_;
    RealTimePlot *angle_plot_;
    RealTimePlot *error_plot_;
    RealTimePlot *motor_plot_;

    // Status indicators
    QLabel *connection_status_;
    QLabel *control_mode_;
    QLabel *system_status_;

    // Colors and styling
    const QString PRIMARY_COLOR = "#1e3a8a";    // Deep blue
    const QString SECONDARY_COLOR = "#3b82f6";  // Lighter blue
    const QString SUCCESS_COLOR = "#10b981";    // Green
    const QString WARNING_COLOR = "#f59e0b";    // Orange
    const QString DANGER_COLOR = "#ef4444";     // Red
    const QString BACKGROUND_COLOR = "#0f172a"; // Dark slate
    const QString CARD_COLOR = "#1e293b";       // Slate 800
    const QString TEXT_COLOR = "#f1f5f9";       // Light text
    const QString ACCENT_COLOR = "#06b6d4";     // Cyan
};
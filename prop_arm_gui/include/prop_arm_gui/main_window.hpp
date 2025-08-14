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
#include <memory>

#include "prop_arm_gui/prop_arm_gui_node.hpp"

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

private:
    void setupUI();
    void setupStyles();
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

    // Monitoring panel components
    QGroupBox *monitor_group_;
    QLabel *arm_angle_value_;
    QLabel *motor_speed_value_;
    QLabel *v_emf_value_;
    QLabel *delta_v_emf_value_;
    QLabel *error_value_;
    QLabel *motor_cmd_value_;
    QProgressBar *angle_progress_;
    QProgressBar *velocity_progress_;

    // Status indicators
    QLabel *connection_status_;
    QLabel *control_mode_;
    QLabel *system_status_;

    // Enhanced aerospace color scheme
    const QString PRIMARY_COLOR = "#1a365d";    // Deep space blue
    const QString SECONDARY_COLOR = "#2563eb";  // Electric blue
    const QString SUCCESS_COLOR = "#00ff88";    // Neon green
    const QString WARNING_COLOR = "#ff8c00";    // Neon orange
    const QString DANGER_COLOR = "#ff3366";     // Neon red
    const QString BACKGROUND_COLOR = "#080a0f"; // Deep dark
    const QString CARD_COLOR = "#1e293b";       // Dark card
    const QString TEXT_COLOR = "#e2e8f0";       // Light text
    const QString ACCENT_COLOR = "#00ccff";     // Cyan accent
};
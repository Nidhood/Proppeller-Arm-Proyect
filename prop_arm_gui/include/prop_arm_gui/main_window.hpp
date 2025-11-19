#pragma once

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMutex>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <deque>
#include <memory>

#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include "prop_arm_gui/chart_base.hpp"
#include "prop_arm_gui/prop_arm_gui_node.hpp"

// Estructura para almacenar datos de control
struct ControlData {
  double timestamp;
  double arm_angle_deg;
  double ref_angle_deg;
  double error_deg;
  double motor_speed_rad_s;
  double pwm_input_us;
  double sim_arm_angle_deg;
  double sim_motor_speed_rad_s;
  double sim_pwm_input_us;
};

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<PropArmGuiNode> node,
                      QWidget *parent = nullptr);
  ~MainWindow();

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
  void setupUbuntuScreenGeometry();
  void setupUI();
  void setupControlTab();
  void setupControlPanel();
  void setupControlCharts();
  void createStatusBar();
  void setupStyles();

  // Métodos para manejo de datos
  void storeControlData(const ControlData &data);
  void clearControlData();
  void updateChartsWithData(const ControlData &data);

  std::shared_ptr<PropArmGuiNode> ros_node_;

  // Almacenamiento de datos de control
  std::deque<ControlData> control_data_;
  QMutex *control_data_mutex_;
  static constexpr size_t MAX_CONTROL_POINTS = 1000;

  // UI Components
  QWidget *central_widget_;
  QTabWidget *tab_widget_;
  QWidget *control_widget_;
  QGroupBox *control_group_;
  AerospaceDataVisualizer *data_visualizer_;

  // Control Charts
  ChartBase *angle_vs_reference_chart_;
  ChartBase *error_chart_;

  // Control Inputs
  QCheckBox *auto_mode_checkbox_;
  QSlider *angle_slider_;
  QDoubleSpinBox *angle_spinbox_;
  QSlider *pwm_slider_;
  QSpinBox *pwm_spinbox_;

  // Step Test Controls
  QDoubleSpinBox *step_angle_low_spinbox_;
  QDoubleSpinBox *step_angle_high_spinbox_;
  QDoubleSpinBox *step_time_up_spinbox_;
  QDoubleSpinBox *step_time_down_spinbox_;
  QPushButton *start_step_test_btn_;
  QPushButton *stop_step_test_btn_;

  // Recording Controls
  QDoubleSpinBox *recording_duration_spinbox_;
  QPushButton *start_recording_btn_;
  QPushButton *stop_recording_btn_;
  QLabel *recording_status_label_;
  QProgressBar *recording_progress_bar_;

  // Action Buttons
  QPushButton *stop_btn_;
  QPushButton *stabilize_btn_;
  QPushButton *refresh_btn_;
  QPushButton *export_btn_;

  // Status Bar
  QLabel *connection_status_;
  QLabel *control_mode_;
  QLabel *system_status_;

  // Update Timer
  QTimer *update_timer_;

  // Modern Dark Theme Colors
  const QString BACKGROUND_COLOR = "#0a0e27";
  const QString CARD_COLOR = "#1a1f3a";
  const QString PRIMARY_COLOR = "#2d3250";
  const QString SECONDARY_COLOR = "#424769";
  const QString TEXT_COLOR = "#e4e4e7";
  const QString ACCENT_COLOR = "#7dd3fc";
  const QString SUCCESS_COLOR = "#4ade80";
  const QString WARNING_COLOR = "#fbbf24";
  const QString DANGER_COLOR = "#f87171";
};
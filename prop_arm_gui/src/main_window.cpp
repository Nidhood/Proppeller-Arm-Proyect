#include "prop_arm_gui/main_window.hpp"
#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include "prop_arm_gui/data_exporter.hpp"
#include "prop_arm_gui/export_preview_dialog.hpp"
#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QMutexLocker>
#include <QScreen>
#include <algorithm>

MainWindow::MainWindow(std::shared_ptr<PropArmGuiNode> node, QWidget *parent)
    : QMainWindow(parent), ros_node_(node), control_data_mutex_(new QMutex()),
      angle_vs_reference_chart_(nullptr), error_chart_(nullptr) {
  setupUbuntuScreenGeometry();
  setupUI();
  setupStyles();

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(50);
  connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateDisplays);
  update_timer_->start();

  if (ros_node_) {
    connect(ros_node_.get(), &PropArmGuiNode::dataUpdated, this,
            &MainWindow::updateDisplays, Qt::QueuedConnection);

    connect(
        ros_node_.get(), &PropArmGuiNode::connectionChanged, this,
        [this](bool connected) {
          connection_status_->setText(connected ? "Connected" : "Disconnected");
          connection_status_->setStyleSheet(
              connected ? QString("color: %1;").arg(SUCCESS_COLOR)
                        : QString("color: %1;").arg(DANGER_COLOR));
        },
        Qt::QueuedConnection);

    connect(ros_node_.get(), &PropArmGuiNode::recordingStarted, this,
            &MainWindow::onRecordingStarted, Qt::QueuedConnection);

    connect(ros_node_.get(), &PropArmGuiNode::recordingProgress, this,
            &MainWindow::onRecordingProgress, Qt::QueuedConnection);

    connect(ros_node_.get(), &PropArmGuiNode::recordingCompleted, this,
            &MainWindow::onRecordingCompleted, Qt::QueuedConnection);
  }

  setWindowTitle("PropArm Control System - Enhanced Control Panel");
}

MainWindow::~MainWindow() {
  if (update_timer_) {
    update_timer_->stop();
  }
  delete control_data_mutex_;
}

void MainWindow::setupUbuntuScreenGeometry() {
  QScreen *screen = QGuiApplication::primaryScreen();
  if (screen) {
    QRect screenGeometry = screen->geometry();
    int width = static_cast<int>(screenGeometry.width() * 0.95);
    int height = static_cast<int>(screenGeometry.height() * 0.90);
    resize(width, height);

    int x = (screenGeometry.width() - width) / 2;
    int y = (screenGeometry.height() - height) / 2;
    move(x, y);
  } else {
    resize(1600, 900);
  }
}

void MainWindow::setupUI() {
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);

  QVBoxLayout *main_layout = new QVBoxLayout(central_widget_);
  main_layout->setSpacing(10);
  main_layout->setContentsMargins(10, 10, 10, 10);

  tab_widget_ = new QTabWidget();
  tab_widget_->setTabPosition(QTabWidget::North);

  setupControlTab();

  data_visualizer_ = new AerospaceDataVisualizer(ros_node_, this);
  tab_widget_->addTab(data_visualizer_, "Real-Time Visualization");

  main_layout->addWidget(tab_widget_);

  createStatusBar();
}

void MainWindow::setupControlTab() {
  control_widget_ = new QWidget();

  QHBoxLayout *main_horizontal_layout = new QHBoxLayout(control_widget_);
  main_horizontal_layout->setSpacing(15);
  main_horizontal_layout->setContentsMargins(10, 10, 10, 10);

  setupControlPanel();
  main_horizontal_layout->addWidget(control_group_, 2);

  setupControlCharts();

  QWidget *charts_container = new QWidget();
  QVBoxLayout *charts_layout = new QVBoxLayout(charts_container);
  charts_layout->setSpacing(10);
  charts_layout->setContentsMargins(0, 0, 0, 0);

  charts_layout->addWidget(angle_vs_reference_chart_);
  charts_layout->addWidget(error_chart_);

  main_horizontal_layout->addWidget(charts_container, 3);

  tab_widget_->addTab(control_widget_, "Control Panel");
}

void MainWindow::setupControlPanel() {
  control_group_ = new QGroupBox("Control Commands");
  QVBoxLayout *control_group_layout = new QVBoxLayout(control_group_);
  control_group_layout->setSpacing(10);

  QGroupBox *mode_group = new QGroupBox("Control Mode");
  QVBoxLayout *mode_layout = new QVBoxLayout(mode_group);
  mode_layout->setSpacing(5);

  auto_mode_checkbox_ = new QCheckBox("Automatic Mode");
  auto_mode_checkbox_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));
  connect(auto_mode_checkbox_, &QCheckBox::toggled, this,
          &MainWindow::onAutoModeToggled);
  mode_layout->addWidget(auto_mode_checkbox_);

  QHBoxLayout *mode_buttons_layout = new QHBoxLayout();
  mode_buttons_layout->setSpacing(5);

  stop_btn_ = new QPushButton("STOP");
  stop_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: "
                                   "white; font-weight: bold; padding: 8px; "
                                   "}")
                               .arg(DANGER_COLOR));

  stabilize_btn_ = new QPushButton("STABILIZE");
  stabilize_btn_->setStyleSheet(
      QString("QPushButton { background-color: %1; color: white; font-weight: "
              "bold; padding: "
              "8px; }")
          .arg(SUCCESS_COLOR));

  mode_buttons_layout->addWidget(stop_btn_);
  mode_buttons_layout->addWidget(stabilize_btn_);

  mode_layout->addLayout(mode_buttons_layout);

  QGroupBox *angle_group = new QGroupBox("Reference Angle");
  QGridLayout *angle_layout = new QGridLayout(angle_group);
  angle_layout->setSpacing(5);

  angle_slider_ = new QSlider(Qt::Horizontal);
  angle_slider_->setRange(0, 90);
  angle_slider_->setValue(0);
  angle_slider_->setTickPosition(QSlider::TicksBelow);
  angle_slider_->setTickInterval(15);

  angle_spinbox_ = new QDoubleSpinBox();
  angle_spinbox_->setRange(-90.0, 90.0);
  angle_spinbox_->setValue(0.0);
  angle_spinbox_->setSuffix(" °");
  angle_spinbox_->setDecimals(1);
  angle_spinbox_->setMaximumWidth(100);

  angle_layout->addWidget(new QLabel("Angle:"), 0, 0);
  angle_layout->addWidget(angle_slider_, 0, 1);
  angle_layout->addWidget(angle_spinbox_, 0, 2);

  QGroupBox *pwm_group = new QGroupBox("PWM Input");
  QGridLayout *pwm_layout = new QGridLayout(pwm_group);
  pwm_layout->setSpacing(5);

  pwm_slider_ = new QSlider(Qt::Horizontal);
  pwm_slider_->setRange(1000, 2000);
  pwm_slider_->setValue(1000);
  pwm_slider_->setTickPosition(QSlider::TicksBelow);
  pwm_slider_->setTickInterval(100);

  pwm_spinbox_ = new QSpinBox();
  pwm_spinbox_->setRange(1000, 2000);
  pwm_spinbox_->setValue(1000);
  pwm_spinbox_->setSuffix(" µs");
  pwm_spinbox_->setMaximumWidth(100);

  pwm_layout->addWidget(new QLabel("PWM:"), 0, 0);
  pwm_layout->addWidget(pwm_slider_, 0, 1);
  pwm_layout->addWidget(pwm_spinbox_, 0, 2);

  QGroupBox *step_group = new QGroupBox("Step Test");
  QGridLayout *step_layout = new QGridLayout(step_group);
  step_layout->setSpacing(5);

  step_angle_low_spinbox_ = new QDoubleSpinBox();
  step_angle_low_spinbox_->setRange(-90.0, 90.0);
  step_angle_low_spinbox_->setValue(0.0);
  step_angle_low_spinbox_->setSuffix(" °");
  step_angle_low_spinbox_->setDecimals(1);

  step_angle_high_spinbox_ = new QDoubleSpinBox();
  step_angle_high_spinbox_->setRange(-90.0, 90.0);
  step_angle_high_spinbox_->setValue(45.0);
  step_angle_high_spinbox_->setSuffix(" °");
  step_angle_high_spinbox_->setDecimals(1);

  step_time_up_spinbox_ = new QDoubleSpinBox();
  step_time_up_spinbox_->setRange(0.1, 60.0);
  step_time_up_spinbox_->setValue(5.0);
  step_time_up_spinbox_->setSuffix(" s");
  step_time_up_spinbox_->setDecimals(1);

  step_time_down_spinbox_ = new QDoubleSpinBox();
  step_time_down_spinbox_->setRange(0.1, 60.0);
  step_time_down_spinbox_->setValue(5.0);
  step_time_down_spinbox_->setSuffix(" s");
  step_time_down_spinbox_->setDecimals(1);

  start_step_test_btn_ = new QPushButton("START");
  start_step_test_btn_->setStyleSheet(
      QString("QPushButton { background-color: %1; color: white; font-weight: "
              "bold; "
              "padding: 6px; }")
          .arg(SUCCESS_COLOR));

  stop_step_test_btn_ = new QPushButton("STOP");
  stop_step_test_btn_->setStyleSheet(
      QString("QPushButton { background-color: %1; color: white; font-weight: "
              "bold; "
              "padding: 6px; }")
          .arg(DANGER_COLOR));
  stop_step_test_btn_->setEnabled(false);

  step_layout->addWidget(new QLabel("Low:"), 0, 0);
  step_layout->addWidget(step_angle_low_spinbox_, 0, 1);
  step_layout->addWidget(new QLabel("High:"), 0, 2);
  step_layout->addWidget(step_angle_high_spinbox_, 0, 3);
  step_layout->addWidget(new QLabel("Time Up:"), 1, 0);
  step_layout->addWidget(step_time_up_spinbox_, 1, 1);
  step_layout->addWidget(new QLabel("Down:"), 1, 2);
  step_layout->addWidget(step_time_down_spinbox_, 1, 3);
  step_layout->addWidget(start_step_test_btn_, 2, 0, 1, 2);
  step_layout->addWidget(stop_step_test_btn_, 2, 2, 1, 2);

  QGroupBox *recording_group = new QGroupBox("Recording");
  QGridLayout *recording_layout = new QGridLayout(recording_group);
  recording_layout->setSpacing(5);

  recording_duration_spinbox_ = new QDoubleSpinBox();
  recording_duration_spinbox_->setRange(5.0, 600.0);
  recording_duration_spinbox_->setValue(120.0);
  recording_duration_spinbox_->setSuffix(" s");
  recording_duration_spinbox_->setDecimals(1);

  start_recording_btn_ = new QPushButton("START REC");
  start_recording_btn_->setStyleSheet(
      QString("QPushButton { background-color: %1; color: white; font-weight: "
              "bold; "
              "padding: 6px; }")
          .arg(SUCCESS_COLOR));

  stop_recording_btn_ = new QPushButton("STOP REC");
  stop_recording_btn_->setStyleSheet(
      QString("QPushButton { background-color: %1; color: white; font-weight: "
              "bold; "
              "padding: 6px; }")
          .arg(WARNING_COLOR));
  stop_recording_btn_->setEnabled(false);

  recording_status_label_ = new QLabel("Not Recording");
  recording_status_label_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(TEXT_COLOR));

  recording_progress_bar_ = new QProgressBar();
  recording_progress_bar_->setRange(0, 100);
  recording_progress_bar_->setValue(0);
  recording_progress_bar_->setFormat("%v s");
  recording_progress_bar_->setVisible(false);

  recording_layout->addWidget(new QLabel("Duration:"), 0, 0);
  recording_layout->addWidget(recording_duration_spinbox_, 0, 1);
  recording_layout->addWidget(start_recording_btn_, 0, 2);
  recording_layout->addWidget(stop_recording_btn_, 0, 3);
  recording_layout->addWidget(recording_status_label_, 1, 0, 1, 4);
  recording_layout->addWidget(recording_progress_bar_, 2, 0, 1, 4);

  QGridLayout *button_layout = new QGridLayout();
  button_layout->setSpacing(5);

  refresh_btn_ = new QPushButton("REFRESH");
  refresh_btn_->setStyleSheet(
      QString("QPushButton { background-color: %1; color: white; font-weight: "
              "bold; padding: "
              "8px; }")
          .arg(SECONDARY_COLOR));

  export_btn_ = new QPushButton("EXPORT");
  export_btn_->setStyleSheet(
      QString("QPushButton { background-color: %1; color: white; font-weight: "
              "bold; padding: "
              "8px; }")
          .arg(ACCENT_COLOR));
  export_btn_->setEnabled(false);

  button_layout->addWidget(refresh_btn_, 0, 0);
  button_layout->addWidget(export_btn_, 0, 1);

  control_group_layout->addWidget(mode_group);
  control_group_layout->addWidget(angle_group);
  control_group_layout->addWidget(pwm_group);
  control_group_layout->addWidget(step_group);
  control_group_layout->addWidget(recording_group);
  control_group_layout->addLayout(button_layout);
  control_group_layout->addStretch();

  connect(angle_slider_, &QSlider::valueChanged, this,
          &MainWindow::onAngleSliderChanged);
  connect(pwm_slider_, &QSlider::valueChanged, this,
          &MainWindow::onPWMSliderChanged);
  connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::onStopClicked);
  connect(stabilize_btn_, &QPushButton::clicked, this,
          &MainWindow::onStabilizeClicked);
  connect(refresh_btn_, &QPushButton::clicked, this,
          &MainWindow::onRefreshClicked);
  connect(export_btn_, &QPushButton::clicked, this,
          &MainWindow::onExportDataClicked);
  connect(start_recording_btn_, &QPushButton::clicked, this,
          &MainWindow::onStartRecordingClicked);
  connect(stop_recording_btn_, &QPushButton::clicked, this,
          &MainWindow::onStopRecordingClicked);
  connect(start_step_test_btn_, &QPushButton::clicked, this,
          &MainWindow::onStartStepTestClicked);
  connect(stop_step_test_btn_, &QPushButton::clicked, this,
          &MainWindow::onStopStepTestClicked);

  connect(angle_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          [this](double value) {
            angle_slider_->setValue(static_cast<int>(value));
          });
  connect(pwm_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged),
          [this](int value) { pwm_slider_->setValue(value); });
}

void MainWindow::setupControlCharts() {
  ChartBase::ChartConfig angle_config;
  angle_config.title = "ANGLE: Real vs Reference";
  angle_config.y_label = "Angle";
  angle_config.units = "degrees";
  angle_config.primary_color = QColor(100, 200, 255);
  angle_config.secondary_color = QColor(150, 220, 255);
  angle_config.sim_color = QColor(255, 150, 100);
  angle_config.y_min = 0.0;
  angle_config.y_max = 90.0;
  angle_config.auto_scale = false;
  angle_config.show_grid = true;
  angle_config.time_window_sec = 30.0;
  angle_config.max_points = 800;
  angle_config.show_milliseconds = false;
  angle_config.use_smooth_curves = true;
  angle_config.show_minor_grid = true;
  angle_config.show_sim_series = true;

  angle_vs_reference_chart_ = new ChartBase(angle_config);
  angle_vs_reference_chart_->setMinimumHeight(300);

  ChartBase::ChartConfig error_config;
  error_config.title = "TRACKING ERROR (Ref - Real)";
  error_config.y_label = "Error";
  error_config.units = "degrees";
  error_config.primary_color = QColor(255, 100, 100);
  error_config.secondary_color = QColor(255, 150, 150);
  error_config.sim_color = QColor(200, 200, 200);
  error_config.y_min = -45.0;
  error_config.y_max = 45.0;
  error_config.auto_scale = false;
  error_config.show_grid = true;
  error_config.time_window_sec = 30.0;
  error_config.max_points = 800;
  error_config.show_milliseconds = false;
  error_config.use_smooth_curves = true;
  error_config.show_minor_grid = true;
  error_config.show_sim_series = false;

  error_chart_ = new ChartBase(error_config);
  error_chart_->setMinimumHeight(300);
}

void MainWindow::createStatusBar() {
  QStatusBar *status = statusBar();

  connection_status_ = new QLabel("Disconnected");
  connection_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));

  control_mode_ = new QLabel("Mode: Manual");
  control_mode_->setStyleSheet(QString("color: %1;").arg(TEXT_COLOR));

  system_status_ = new QLabel("System: Ready");
  system_status_->setStyleSheet(QString("color: %1;").arg(SUCCESS_COLOR));

  status->addPermanentWidget(connection_status_);
  status->addPermanentWidget(new QLabel(" | "));
  status->addPermanentWidget(control_mode_);
  status->addPermanentWidget(new QLabel(" | "));
  status->addPermanentWidget(system_status_);
}

void MainWindow::setupStyles() {
  QString stylesheet = QString(R"(
        QMainWindow {
            background-color: %1;
        }
        QGroupBox {
            border: 2px solid %2;
            border-radius: 8px;
            margin-top: 12px;
            padding-top: 10px;
            font-weight: bold;
            color: %3;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 15px;
            padding: 0 5px;
        }
        QLabel {
            color: %3;
        }
        QSlider::groove:horizontal {
            height: 8px;
            background: %4;
            border-radius: 4px;
        }
        QSlider::handle:horizontal {
            background: %5;
            width: 18px;
            margin: -5px 0;
            border-radius: 9px;
        }
        QSlider::handle:horizontal:hover {
            background: %6;
        }
        QProgressBar {
            border: 2px solid %2;
            border-radius: 5px;
            text-align: center;
            background-color: %4;
            color: %3;
        }
        QProgressBar::chunk {
            background-color: %5;
            border-radius: 3px;
        }
        QSpinBox, QDoubleSpinBox {
            background-color: %4;
            border: 1px solid %2;
            border-radius: 4px;
            padding: 5px;
            color: %3;
        }
        QTabWidget::pane {
            border: 2px solid %2;
            border-radius: 8px;
            background-color: %1;
        }
        QTabBar::tab {
            background-color: %4;
            color: %3;
            padding: 10px 20px;
            border: 1px solid %2;
            border-bottom: none;
            border-top-left-radius: 8px;
            border-top-right-radius: 8px;
        }
        QTabBar::tab:selected {
            background-color: %2;
            color: %6;
        }
        QStatusBar {
            background-color: %4;
            color: %3;
        }
        QCheckBox {
            color: %3;
        }
    )")
                           .arg(BACKGROUND_COLOR)
                           .arg(CARD_COLOR)
                           .arg(TEXT_COLOR)
                           .arg(PRIMARY_COLOR)
                           .arg(SECONDARY_COLOR)
                           .arg(ACCENT_COLOR);

  setStyleSheet(stylesheet);
}

void MainWindow::storeControlData(const ControlData &data) {
  QMutexLocker locker(control_data_mutex_);

  control_data_.push_back(data);

  while (control_data_.size() > MAX_CONTROL_POINTS) {
    control_data_.pop_front();
  }
}

void MainWindow::clearControlData() {
  QMutexLocker locker(control_data_mutex_);
  control_data_.clear();
}

void MainWindow::updateChartsWithData(const ControlData &data) {
  if (angle_vs_reference_chart_) {
    angle_vs_reference_chart_->addDataPoint(data.arm_angle_deg, data.timestamp);
    angle_vs_reference_chart_->addSimDataPoint(data.ref_angle_deg,
                                               data.timestamp);
  }

  if (error_chart_) {
    error_chart_->addDataPoint(data.error_deg, data.timestamp);
  }
}

void MainWindow::updateDisplays() {
  if (!ros_node_)
    return;

  PropArmData data = ros_node_->getCurrentData();

  if (!data.valid)
    return;

  control_mode_->setText("Mode: " +
                         QString::fromStdString(ros_node_->getControlMode()));

  double timestamp = data.datetime.toMSecsSinceEpoch() / 1000.0;

  ControlData ctrl_data;
  ctrl_data.timestamp = timestamp;
  ctrl_data.arm_angle_deg = data.arm_angle_deg;
  ctrl_data.ref_angle_deg = data.ref_angle_deg;
  ctrl_data.error_deg = data.ref_angle_deg - data.arm_angle_deg;
  ctrl_data.motor_speed_rad_s = data.motor_speed_rad_s;
  ctrl_data.pwm_input_us = data.pwm_input_us;
  ctrl_data.sim_arm_angle_deg = data.sim_arm_angle_deg;
  ctrl_data.sim_motor_speed_rad_s = data.sim_motor_speed_rad_s;
  ctrl_data.sim_pwm_input_us = data.sim_pwm_input_us;

  storeControlData(ctrl_data);
  updateChartsWithData(ctrl_data);

  if (data_visualizer_) {
    data_visualizer_->onDataReceived(data.arm_angle_deg, data.motor_speed_rad_s,
                                     data.pwm_input_us, data.sim_arm_angle_deg,
                                     data.sim_motor_speed_rad_s,
                                     data.sim_pwm_input_us);
  }
}

void MainWindow::onAngleSliderChanged(int value) {
  angle_spinbox_->setValue(static_cast<double>(value));

  if (ros_node_ && !auto_mode_checkbox_->isChecked()) {
    double angle_rad = static_cast<double>(value) * M_PI / 180.0;
    ros_node_->sendAngleCommand(angle_rad);
  }
}

void MainWindow::onPWMSliderChanged(int value) {
  pwm_spinbox_->setValue(value);

  if (ros_node_ && !auto_mode_checkbox_->isChecked()) {
    ros_node_->sendPWMCommand(static_cast<uint16_t>(value));
  }
}

void MainWindow::onAutoModeToggled(bool checked) {
  if (ros_node_) {
    ros_node_->sendAutoModeCommand(checked);
  }

  angle_slider_->setEnabled(!checked);
  pwm_slider_->setEnabled(!checked);
  angle_spinbox_->setEnabled(!checked);
  pwm_spinbox_->setEnabled(!checked);

  system_status_->setText(checked ? "System: Automatic Mode"
                                  : "System: Manual Mode");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;")
          .arg(checked ? SUCCESS_COLOR : WARNING_COLOR));
}

void MainWindow::onStopClicked() {
  if (ros_node_) {
    ros_node_->sendStopCommand();
    ros_node_->stopStepTest();
  }

  angle_slider_->setValue(0);
  pwm_slider_->setValue(1000);
  stop_step_test_btn_->setEnabled(false);
  start_step_test_btn_->setEnabled(true);

  system_status_->setText("System: STOPPED");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));
}

void MainWindow::onStabilizeClicked() {
  if (ros_node_) {
    PropArmData data = ros_node_->getCurrentData();
    double current_angle = data.arm_angle_deg;
    double current_angle_rad = current_angle * M_PI / 180.0;

    angle_slider_->setValue(static_cast<int>(current_angle));
    ros_node_->sendAngleCommand(current_angle_rad);
  }

  system_status_->setText("System: Stabilizing");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(WARNING_COLOR));
}

void MainWindow::onRefreshClicked() {
  if (!ros_node_)
    return;

  size_t recorded_count = ros_node_->getRecordedPointCount();

  if (recorded_count == 0) {
    return;
  }

  if (ros_node_->isRecording()) {
    ros_node_->stopRecording();
  }

  ros_node_->clearRecordedData();

  clearControlData();

  recording_status_label_->setText("Not Recording");
  recording_status_label_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(TEXT_COLOR));

  recording_progress_bar_->setVisible(false);
  recording_progress_bar_->setValue(0);

  start_recording_btn_->setEnabled(true);
  stop_recording_btn_->setEnabled(false);
  export_btn_->setEnabled(false);

  system_status_->setText("System: Refreshed");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));
}

void MainWindow::onExportDataClicked() {
  if (!ros_node_)
    return;

  size_t point_count = ros_node_->getRecordedPointCount();
  if (point_count == 0) {
    QMessageBox::warning(this, "No Data",
                         "No recorded data available to export.");
    return;
  }

  std::vector<PropArmData> recorded_data = ros_node_->getRecordedData();

  ExportPreviewDialog preview_dialog(recorded_data, this);

  if (preview_dialog.exec() == QDialog::Accepted &&
      preview_dialog.wasAccepted()) {
    std::vector<std::string> selected_columns =
        preview_dialog.getSelectedColumns();

    if (selected_columns.empty()) {
      QMessageBox::warning(this, "No Columns Selected",
                           "Please select at least one column to export.");
      return;
    }

    QString default_filename =
        QString("proparm_recording_%1.csv")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    QString filename = QFileDialog::getSaveFileName(
        this, "Export Recorded Data", QDir::homePath() + "/" + default_filename,
        "CSV Files (*.csv);;All Files (*)");

    if (filename.isEmpty())
      return;

    DataExporter exporter;
    double sim_start = ros_node_->getSimulationStartTime();

    if (exporter.exportToCSV(filename, recorded_data, sim_start,
                             selected_columns)) {
      QMessageBox::information(
          this, "Export Complete",
          QString(
              "Successfully exported %1 data points with %2 columns to:\n%3")
              .arg(point_count)
              .arg(selected_columns.size())
              .arg(filename));
    } else {
      QMessageBox::critical(
          this, "Export Failed",
          QString("Failed to export data:\n%1").arg(exporter.getLastError()));
    }
  }
}

void MainWindow::onStartRecordingClicked() {
  if (!ros_node_)
    return;

  double duration = recording_duration_spinbox_->value();
  ros_node_->startRecording(duration);
}

void MainWindow::onStopRecordingClicked() {
  if (!ros_node_)
    return;

  ros_node_->stopRecording();
}

void MainWindow::onRecordingStarted(double duration) {
  recording_status_label_->setText("Recording...");
  recording_status_label_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));

  start_recording_btn_->setEnabled(false);
  stop_recording_btn_->setEnabled(true);
  export_btn_->setEnabled(false);

  recording_progress_bar_->setVisible(true);
  recording_progress_bar_->setMaximum(static_cast<int>(duration));
  recording_progress_bar_->setValue(static_cast<int>(duration));

  system_status_->setText("System: Recording Data");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));
}

void MainWindow::onRecordingProgress(double remaining_time,
                                     size_t point_count) {
  recording_progress_bar_->setValue(static_cast<int>(remaining_time));

  recording_status_label_->setText(
      QString("Recording... %1 pts").arg(point_count));
}

void MainWindow::onRecordingCompleted(size_t point_count, double duration) {
  recording_status_label_->setText(QString("Complete: %1 pts (%2s)")
                                       .arg(point_count)
                                       .arg(duration, 0, 'f', 1));
  recording_status_label_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));

  start_recording_btn_->setEnabled(true);
  stop_recording_btn_->setEnabled(false);
  export_btn_->setEnabled(point_count > 0);

  recording_progress_bar_->setVisible(false);

  system_status_->setText("System: Recording Complete");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));

  if (point_count > 0) {
    QMessageBox::information(
        this, "Recording Complete",
        QString("Successfully recorded %1 data points over %2 seconds.\n\n"
                "Click 'EXPORT' to save to CSV.")
            .arg(point_count)
            .arg(duration, 0, 'f', 1));
  }
}

void MainWindow::onStartStepTestClicked() {
  if (!ros_node_)
    return;

  double angle_low = step_angle_low_spinbox_->value();
  double angle_high = step_angle_high_spinbox_->value();
  double time_up = step_time_up_spinbox_->value();
  double time_down = step_time_down_spinbox_->value();

  ros_node_->startStepTest(angle_low, angle_high, time_up, time_down);

  start_step_test_btn_->setEnabled(false);
  stop_step_test_btn_->setEnabled(true);

  system_status_->setText("System: Step Test Running");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));
}

void MainWindow::onStopStepTestClicked() {
  if (!ros_node_)
    return;

  ros_node_->stopStepTest();

  start_step_test_btn_->setEnabled(true);
  stop_step_test_btn_->setEnabled(false);

  system_status_->setText("System: Step Test Stopped");
  system_status_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(WARNING_COLOR));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto options = rclcpp::NodeOptions().arguments(std::vector<std::string>{});
  auto node = std::make_shared<PropArmGuiNode>(options);

  MainWindow window(node);
  window.showMaximized();

  std::thread ros_thread([&node]() { rclcpp::spin(node); });

  int result = app.exec();

  rclcpp::shutdown();
  ros_thread.join();

  return result;
}

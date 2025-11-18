#include "prop_arm_gui/main_window.hpp"
#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include "prop_arm_gui/data_exporter.hpp"
#include <QApplication>
#include <QScreen>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>
#include <QDir>

MainWindow::MainWindow(std::shared_ptr<PropArmGuiNode> node, QWidget *parent)
    : QMainWindow(parent), ros_node_(node)
{
    setupUbuntuScreenGeometry();
    setupUI();
    setupStyles();

    update_timer_ = new QTimer(this);
    update_timer_->setInterval(50);
    connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateDisplays);
    update_timer_->start();

    if (ros_node_)
    {
        connect(ros_node_.get(), &PropArmGuiNode::dataUpdated,
                this, &MainWindow::updateDisplays, Qt::QueuedConnection);

        connect(ros_node_.get(), &PropArmGuiNode::connectionChanged, this, [this](bool connected)
                {
            connection_status_->setText(connected ? "Connected" : "Disconnected");
            connection_status_->setStyleSheet(connected ? 
                QString("color: %1;").arg(SUCCESS_COLOR) : 
                QString("color: %1;").arg(DANGER_COLOR)); }, Qt::QueuedConnection);

        // Señales de grabación
        connect(ros_node_.get(), &PropArmGuiNode::recordingStarted,
                this, &MainWindow::onRecordingStarted, Qt::QueuedConnection);

        connect(ros_node_.get(), &PropArmGuiNode::recordingProgress,
                this, &MainWindow::onRecordingProgress, Qt::QueuedConnection);

        connect(ros_node_.get(), &PropArmGuiNode::recordingCompleted,
                this, &MainWindow::onRecordingCompleted, Qt::QueuedConnection);
    }

    setWindowTitle("PropArm Control System - PWM & Duty Cycle Monitoring");
}

void MainWindow::setupUbuntuScreenGeometry()
{
    QScreen *screen = QGuiApplication::primaryScreen();
    if (screen)
    {
        QRect screenGeometry = screen->geometry();
        int width = static_cast<int>(screenGeometry.width() * 0.95);
        int height = static_cast<int>(screenGeometry.height() * 0.90);
        resize(width, height);

        int x = (screenGeometry.width() - width) / 2;
        int y = (screenGeometry.height() - height) / 2;
        move(x, y);
    }
    else
    {
        resize(1600, 900);
    }
}

void MainWindow::setupUI()
{
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);

    QVBoxLayout *main_layout = new QVBoxLayout(central_widget_);
    main_layout->setSpacing(10);
    main_layout->setContentsMargins(10, 10, 10, 10);

    // Tab widget
    tab_widget_ = new QTabWidget();
    tab_widget_->setTabPosition(QTabWidget::North);

    // Control tab
    setupControlTab();

    // Visualization tab
    data_visualizer_ = new AerospaceDataVisualizer(ros_node_, this);
    tab_widget_->addTab(data_visualizer_, "Real-Time Visualization");

    main_layout->addWidget(tab_widget_);

    createStatusBar();
}

void MainWindow::setupControlTab()
{
    control_widget_ = new QWidget();
    QHBoxLayout *control_layout = new QHBoxLayout(control_widget_);
    control_layout->setSpacing(15);
    control_layout->setContentsMargins(10, 10, 10, 10);

    setupControlPanel();
    setupMonitoringPanel();

    control_layout->addWidget(control_group_, 1);
    control_layout->addWidget(monitor_group_, 1);

    tab_widget_->addTab(control_widget_, "Control Panel");
}

void MainWindow::setupControlPanel()
{
    control_group_ = new QGroupBox("Control Commands");
    QVBoxLayout *control_group_layout = new QVBoxLayout(control_group_);

    // Angle Control Section
    QGroupBox *angle_group = new QGroupBox("Angle Control");
    QGridLayout *angle_layout = new QGridLayout(angle_group);

    angle_slider_ = new QSlider(Qt::Horizontal);
    angle_slider_->setRange(-90, 90);
    angle_slider_->setValue(0);
    angle_slider_->setTickPosition(QSlider::TicksBelow);
    angle_slider_->setTickInterval(30);

    angle_spinbox_ = new QDoubleSpinBox();
    angle_spinbox_->setRange(-90.0, 90.0);
    angle_spinbox_->setValue(0.0);
    angle_spinbox_->setSuffix(" °");
    angle_spinbox_->setDecimals(1);

    angle_layout->addWidget(new QLabel("Target Angle:"), 0, 0);
    angle_layout->addWidget(angle_slider_, 0, 1);
    angle_layout->addWidget(angle_spinbox_, 0, 2);

    // Velocity Control Section
    QGroupBox *velocity_group = new QGroupBox("Velocity Control");
    QGridLayout *velocity_layout = new QGridLayout(velocity_group);

    velocity_slider_ = new QSlider(Qt::Horizontal);
    velocity_slider_->setRange(-785, 785);
    velocity_slider_->setValue(0);
    velocity_slider_->setTickPosition(QSlider::TicksBelow);
    velocity_slider_->setTickInterval(100);

    velocity_spinbox_ = new QDoubleSpinBox();
    velocity_spinbox_->setRange(-785.0, 785.0);
    velocity_spinbox_->setValue(0.0);
    velocity_spinbox_->setSuffix(" rad/s");
    velocity_spinbox_->setDecimals(1);

    velocity_layout->addWidget(new QLabel("Target Velocity:"), 0, 0);
    velocity_layout->addWidget(velocity_slider_, 0, 1);
    velocity_layout->addWidget(velocity_spinbox_, 0, 2);

    // Recording Section
    QGroupBox *recording_group = new QGroupBox("Data Recording");
    QGridLayout *recording_layout = new QGridLayout(recording_group);

    recording_duration_spinbox_ = new QDoubleSpinBox();
    recording_duration_spinbox_->setRange(5.0, 600.0);
    recording_duration_spinbox_->setValue(120.0);
    recording_duration_spinbox_->setSuffix(" s");
    recording_duration_spinbox_->setDecimals(1);

    start_recording_btn_ = new QPushButton("START RECORDING");
    start_recording_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(SUCCESS_COLOR));

    stop_recording_btn_ = new QPushButton("STOP RECORDING");
    stop_recording_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(WARNING_COLOR));
    stop_recording_btn_->setEnabled(false);

    recording_status_label_ = new QLabel("Not Recording");
    recording_status_label_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(TEXT_COLOR));

    recording_progress_bar_ = new QProgressBar();
    recording_progress_bar_->setRange(0, 100);
    recording_progress_bar_->setValue(0);
    recording_progress_bar_->setFormat("%v s remaining");
    recording_progress_bar_->setVisible(false);

    recording_layout->addWidget(new QLabel("Duration:"), 0, 0);
    recording_layout->addWidget(recording_duration_spinbox_, 0, 1);
    recording_layout->addWidget(start_recording_btn_, 0, 2);
    recording_layout->addWidget(stop_recording_btn_, 0, 3);
    recording_layout->addWidget(new QLabel("Status:"), 1, 0);
    recording_layout->addWidget(recording_status_label_, 1, 1, 1, 2);
    recording_layout->addWidget(recording_progress_bar_, 2, 0, 1, 4);

    // Control Buttons
    QHBoxLayout *button_layout = new QHBoxLayout();

    stop_btn_ = new QPushButton("STOP");
    stop_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(DANGER_COLOR));

    stabilize_btn_ = new QPushButton("STABILIZE");
    stabilize_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(SUCCESS_COLOR));

    refresh_btn_ = new QPushButton("REFRESH");
    refresh_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(SECONDARY_COLOR));

    export_btn_ = new QPushButton("EXPORT RECORDED DATA");
    export_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(ACCENT_COLOR));
    export_btn_->setEnabled(false);

    button_layout->addWidget(stop_btn_);
    button_layout->addWidget(stabilize_btn_);
    button_layout->addWidget(refresh_btn_);
    button_layout->addWidget(export_btn_);

    // Add to control group
    control_group_layout->addWidget(angle_group);
    control_group_layout->addWidget(velocity_group);
    control_group_layout->addWidget(recording_group);
    control_group_layout->addLayout(button_layout);
    control_group_layout->addStretch();

    // Connect signals
    connect(angle_slider_, &QSlider::valueChanged, this, &MainWindow::onAngleSliderChanged);
    connect(velocity_slider_, &QSlider::valueChanged, this, &MainWindow::onVelocitySliderChanged);
    connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(stabilize_btn_, &QPushButton::clicked, this, &MainWindow::onStabilizeClicked);
    connect(refresh_btn_, &QPushButton::clicked, this, &MainWindow::onRefreshClicked);
    connect(export_btn_, &QPushButton::clicked, this, &MainWindow::onExportDataClicked);

    // Recording signals
    connect(start_recording_btn_, &QPushButton::clicked, this, &MainWindow::onStartRecordingClicked);
    connect(stop_recording_btn_, &QPushButton::clicked, this, &MainWindow::onStopRecordingClicked);

    // Sync spinboxes with sliders
    connect(angle_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value)
            { angle_slider_->setValue(static_cast<int>(value)); });
    connect(velocity_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value)
            { velocity_slider_->setValue(static_cast<int>(value)); });
}

void MainWindow::setupMonitoringPanel()
{
    monitor_group_ = new QGroupBox("System Monitoring");
    QVBoxLayout *monitor_layout = new QVBoxLayout(monitor_group_);

    // Create grid for values
    QGridLayout *values_grid = new QGridLayout();

    // Arm Angle
    values_grid->addWidget(new QLabel("Arm Angle:"), 0, 0);
    arm_angle_value_ = new QLabel("0.0 °");
    arm_angle_value_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 14pt;").arg(ACCENT_COLOR));
    values_grid->addWidget(arm_angle_value_, 0, 1);

    angle_progress_ = new QProgressBar();
    angle_progress_->setRange(-90, 90);
    angle_progress_->setValue(0);
    angle_progress_->setFormat("%v°");
    values_grid->addWidget(angle_progress_, 0, 2);

    // Motor Speed
    values_grid->addWidget(new QLabel("Motor Speed:"), 1, 0);
    motor_speed_value_ = new QLabel("0.0 rad/s");
    motor_speed_value_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 14pt;").arg(SUCCESS_COLOR));
    values_grid->addWidget(motor_speed_value_, 1, 1);

    velocity_progress_ = new QProgressBar();
    velocity_progress_->setRange(-785, 785);
    velocity_progress_->setValue(0);
    velocity_progress_->setFormat("%v rad/s");
    values_grid->addWidget(velocity_progress_, 1, 2);

    // PWM Input
    values_grid->addWidget(new QLabel("PWM Input:"), 2, 0);
    pwm_input_value_ = new QLabel("0 µs");
    pwm_input_value_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 14pt;").arg(WARNING_COLOR));
    values_grid->addWidget(pwm_input_value_, 2, 1);

    // Duty Cycle
    values_grid->addWidget(new QLabel("Duty Cycle:"), 3, 0);
    duty_cycle_value_ = new QLabel("0.00 %");
    duty_cycle_value_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 14pt;").arg(ACCENT_COLOR));
    values_grid->addWidget(duty_cycle_value_, 3, 1);

    // Control Error
    values_grid->addWidget(new QLabel("Control Error:"), 4, 0);
    error_value_ = new QLabel("0.0 °");
    error_value_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 14pt;").arg(TEXT_COLOR));
    values_grid->addWidget(error_value_, 4, 1);

    // Motor Command
    values_grid->addWidget(new QLabel("Motor Command:"), 5, 0);
    motor_cmd_value_ = new QLabel("0.0 rad/s");
    motor_cmd_value_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 14pt;").arg(TEXT_COLOR));
    values_grid->addWidget(motor_cmd_value_, 5, 1);

    monitor_layout->addLayout(values_grid);
    monitor_layout->addStretch();
}

void MainWindow::createStatusBar()
{
    QStatusBar *status = statusBar();

    connection_status_ = new QLabel("Disconnected");
    connection_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));

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

void MainWindow::setupStyles()
{
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
    )")
                             .arg(BACKGROUND_COLOR)
                             .arg(CARD_COLOR)
                             .arg(TEXT_COLOR)
                             .arg(PRIMARY_COLOR)
                             .arg(SECONDARY_COLOR)
                             .arg(ACCENT_COLOR);

    setStyleSheet(stylesheet);
}

void MainWindow::updateDisplays()
{
    if (!ros_node_)
        return;

    PropArmData data = ros_node_->getCurrentData();

    if (!data.valid)
        return;

    // Update values (show REAL data in UI)
    arm_angle_value_->setText(QString::number(data.arm_angle_deg, 'f', 2) + " °");
    motor_speed_value_->setText(QString::number(data.motor_speed_rad_s, 'f', 2) + " rad/s");
    pwm_input_value_->setText(QString::number(data.pwm_input_us, 'f', 0) + " µs");
    duty_cycle_value_->setText(QString::number(data.duty_cycle_percent, 'f', 2) + " %");
    error_value_->setText(QString::number(data.error, 'f', 2) + " °");
    motor_cmd_value_->setText(QString::number(data.motor_command, 'f', 2) + " rad/s");

    // Update progress bars
    angle_progress_->setValue(static_cast<int>(data.arm_angle_deg));
    velocity_progress_->setValue(static_cast<int>(data.motor_speed_rad_s));

    // Update control mode
    control_mode_->setText("Mode: " + QString::fromStdString(ros_node_->getControlMode()));

    // Update visualizer with BOTH real and simulation data
    if (data_visualizer_)
    {
        data_visualizer_->onDataReceived(
            data.arm_angle_deg,
            data.motor_speed_rad_s,
            data.pwm_input_us,
            data.sim_arm_angle_deg,     // NEW: simulation data
            data.sim_motor_speed_rad_s, // NEW: simulation data
            data.sim_pwm_input_us       // NEW: simulation data
        );
    }
}

void MainWindow::onAngleSliderChanged(int value)
{
    angle_spinbox_->setValue(static_cast<double>(value));

    if (ros_node_)
    {
        ros_node_->sendAngleCommand(static_cast<double>(value));
    }
}

void MainWindow::onVelocitySliderChanged(int value)
{
    velocity_spinbox_->setValue(static_cast<double>(value));

    if (ros_node_)
    {
        ros_node_->sendVelocityCommand(static_cast<double>(value));
    }
}

void MainWindow::onStopClicked()
{
    if (ros_node_)
    {
        ros_node_->sendStopCommand();
    }

    angle_slider_->setValue(0);
    velocity_slider_->setValue(0);

    system_status_->setText("System: STOPPED");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));
}

void MainWindow::onStabilizeClicked()
{
    if (ros_node_)
    {
        PropArmData data = ros_node_->getCurrentData();
        double current_angle = data.arm_angle_deg;

        angle_slider_->setValue(static_cast<int>(current_angle));
        ros_node_->sendAngleCommand(current_angle);
    }

    system_status_->setText("System: Stabilizing");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(WARNING_COLOR));
}

void MainWindow::onRefreshClicked()
{
    if (data_visualizer_)
    {
        data_visualizer_->clearData();
    }

    system_status_->setText("System: Refreshed");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));
}

void MainWindow::onExportDataClicked()
{
    if (!ros_node_)
        return;

    size_t point_count = ros_node_->getRecordedPointCount();
    if (point_count == 0)
    {
        QMessageBox::warning(this, "No Data",
                             "No recorded data available to export.");
        return;
    }

    QString default_filename = QString("proparm_recording_%1.csv")
                                   .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    QString filename = QFileDialog::getSaveFileName(
        this,
        "Export Recorded Data",
        QDir::homePath() + "/" + default_filename,
        "CSV Files (*.csv);;All Files (*)");

    if (filename.isEmpty())
        return;

    DataExporter exporter;
    std::vector<PropArmData> recorded_data = ros_node_->getRecordedData();
    double sim_start = ros_node_->getSimulationStartTime();

    if (exporter.exportToCSV(filename, recorded_data, sim_start))
    {
        QMessageBox::information(this, "Export Complete",
                                 QString("Successfully exported %1 data points to:\n%2")
                                     .arg(point_count)
                                     .arg(filename));
    }
    else
    {
        QMessageBox::critical(this, "Export Failed",
                              QString("Failed to export data:\n%1")
                                  .arg(exporter.getLastError()));
    }
}

void MainWindow::onStartRecordingClicked()
{
    if (!ros_node_)
        return;

    double duration = recording_duration_spinbox_->value();
    ros_node_->startRecording(duration);
}

void MainWindow::onStopRecordingClicked()
{
    if (!ros_node_)
        return;

    ros_node_->stopRecording();
}

void MainWindow::onRecordingStarted(double duration)
{
    recording_status_label_->setText("Recording...");
    recording_status_label_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));

    start_recording_btn_->setEnabled(false);
    stop_recording_btn_->setEnabled(true);
    export_btn_->setEnabled(false);

    recording_progress_bar_->setVisible(true);
    recording_progress_bar_->setMaximum(static_cast<int>(duration));
    recording_progress_bar_->setValue(static_cast<int>(duration));

    system_status_->setText("System: Recording Data");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));
}

void MainWindow::onRecordingProgress(double remaining_time, size_t point_count)
{
    recording_progress_bar_->setValue(static_cast<int>(remaining_time));

    recording_status_label_->setText(
        QString("Recording... %1 points captured")
            .arg(point_count));
}

void MainWindow::onRecordingCompleted(size_t point_count, double duration)
{
    recording_status_label_->setText(
        QString("Recording Complete: %1 points in %2 seconds")
            .arg(point_count)
            .arg(duration, 0, 'f', 1));
    recording_status_label_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));

    start_recording_btn_->setEnabled(true);
    stop_recording_btn_->setEnabled(false);
    export_btn_->setEnabled(point_count > 0);

    recording_progress_bar_->setVisible(false);

    system_status_->setText("System: Recording Complete");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));

    if (point_count > 0)
    {
        QMessageBox::information(this, "Recording Complete",
                                 QString("Successfully recorded %1 data points over %2 seconds.\n\n"
                                         "Click 'EXPORT RECORDED DATA' to save to CSV.")
                                     .arg(point_count)
                                     .arg(duration, 0, 'f', 1));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    auto options = rclcpp::NodeOptions().arguments(std::vector<std::string>{});
    auto node = std::make_shared<PropArmGuiNode>(options);

    MainWindow window(node);
    window.show();

    std::thread ros_thread([&node]()
                           { rclcpp::spin(node); });

    int result = app.exec();

    rclcpp::shutdown();
    ros_thread.join();

    return result;
}
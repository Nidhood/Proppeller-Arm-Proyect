#include "prop_arm_gui/main_window.hpp"
#include <QApplication>
#include <QStatusBar>
#include <QMenuBar>
#include <QToolBar>
#include <QSplitter>
#include <QScrollArea>
#include <QMessageBox>
#include <QFileDialog>
#include <QSettings>

MainWindow::MainWindow(std::shared_ptr<PropArmGuiNode> node, QWidget *parent)
    : QMainWindow(parent), ros_node_(node)
{
    setWindowTitle("PropArm Control System - Aerospace GUI");
    setMinimumSize(1200, 800);
    resize(1600, 1000);

    setupStyles();
    setupUI();

    // Connect ROS node signals
    connect(ros_node_.get(), &PropArmGuiNode::dataUpdated,
            this, &MainWindow::updateDisplays);
    connect(ros_node_.get(), &PropArmGuiNode::connectionChanged,
            this, [this](bool connected)
            {
                connection_status_->setText(connected ? "CONNECTED" : "DISCONNECTED");
                connection_status_->setStyleSheet(connected ? 
                    QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR) :
                    QString("color: %1; font-weight: bold;").arg(DANGER_COLOR)); });

    // Setup update timer
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateDisplays);
    update_timer_->start(50); // 20 FPS

    // Initial update
    updateDisplays();
}

void MainWindow::setupUI()
{
    central_widget_ = new QWidget;
    setCentralWidget(central_widget_);

    main_layout_ = new QGridLayout(central_widget_);
    main_layout_->setSpacing(10);
    main_layout_->setContentsMargins(10, 10, 10, 10);

    setupControlPanel();
    setupMonitoringPanel();
    setupPlotsPanel();
    createStatusBar();

    // Create aerospace dashboard
    dashboard_ = new AerospaceDashboard(this);
    dashboard_->setMinimumSize(400, 400);

    // Layout arrangement
    main_layout_->addWidget(control_group_, 0, 0, 1, 1);
    main_layout_->addWidget(monitor_group_, 0, 1, 1, 1);
    main_layout_->addWidget(dashboard_, 1, 0, 1, 2);
    main_layout_->addWidget(plots_group_, 2, 0, 1, 2);

    // Set column and row stretches
    main_layout_->setColumnStretch(0, 1);
    main_layout_->setColumnStretch(1, 1);
    main_layout_->setRowStretch(0, 0);
    main_layout_->setRowStretch(1, 1);
    main_layout_->setRowStretch(2, 1);
}

void MainWindow::setupStyles()
{
    QString main_style = QString(R"(
        QMainWindow {
            background-color: %1;
            color: %2;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid %3;
            border-radius: 8px;
            margin-top: 1ex;
            padding-top: 10px;
            background-color: %4;
            color: %2;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 8px 0 8px;
            color: %5;
            font-size: 12pt;
        }
        QPushButton {
            background-color: %3;
            border: 2px solid %5;
            border-radius: 6px;
            padding: 8px 16px;
            font-weight: bold;
            color: %2;
            min-width: 80px;
        }
        QPushButton:hover {
            background-color: %5;
            color: %1;
        }
        QPushButton:pressed {
            background-color: %6;
        }
        QPushButton#stopButton {
            background-color: %7;
            border-color: %7;
        }
        QPushButton#stopButton:hover {
            background-color: #dc2626;
        }
        QSlider::groove:horizontal {
            border: 1px solid %3;
            height: 8px;
            background: %1;
            border-radius: 4px;
        }
        QSlider::handle:horizontal {
            background: %5;
            border: 2px solid %3;
            width: 20px;
            height: 20px;
            border-radius: 12px;
            margin: -6px 0;
        }
        QSlider::handle:horizontal:hover {
            background: %6;
        }
        QProgressBar {
            border: 2px solid %3;
            border-radius: 6px;
            background-color: %1;
            text-align: center;
            font-weight: bold;
            color: %2;
        }
        QProgressBar::chunk {
            background-color: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                stop: 0 %5, stop: 1 %6);
            border-radius: 4px;
        }
        QLabel {
            color: %2;
            font-size: 11pt;
        }
        QLabel#valueLabel {
            font-size: 14pt;
            font-weight: bold;
            color: %5;
            background-color: %1;
            border: 1px solid %3;
            border-radius: 4px;
            padding: 4px 8px;
        }
        QSpinBox, QDoubleSpinBox {
            background-color: %1;
            border: 2px solid %3;
            border-radius: 4px;
            padding: 4px;
            color: %2;
            font-weight: bold;
        }
        QSpinBox:focus, QDoubleSpinBox:focus {
            border-color: %5;
        }
    )")
                             .arg(BACKGROUND_COLOR, TEXT_COLOR, PRIMARY_COLOR, CARD_COLOR,
                                  ACCENT_COLOR, SECONDARY_COLOR, DANGER_COLOR);

    setStyleSheet(main_style);
}

void MainWindow::setupControlPanel()
{
    control_group_ = new QGroupBox("Control Panel");
    auto layout = new QVBoxLayout(control_group_);

    // Angle control
    auto angle_layout = new QHBoxLayout();
    angle_layout->addWidget(new QLabel("Angle (°):"));
    angle_slider_ = new QSlider(Qt::Horizontal);
    angle_slider_->setRange(-90, 90);
    angle_slider_->setValue(0);
    angle_spinbox_ = new QDoubleSpinBox();
    angle_spinbox_->setRange(-90.0, 90.0);
    angle_spinbox_->setSuffix(" °");
    angle_spinbox_->setValue(0.0);
    angle_layout->addWidget(angle_slider_);
    angle_layout->addWidget(angle_spinbox_);

    // Force control
    auto force_layout = new QHBoxLayout();
    force_layout->addWidget(new QLabel("Force (N):"));
    force_slider_ = new QSlider(Qt::Horizontal);
    force_slider_->setRange(0, 450); // 0-45.0 N with 0.1 resolution
    force_slider_->setValue(0);
    force_spinbox_ = new QDoubleSpinBox();
    force_spinbox_->setRange(0.0, 45.0);
    force_spinbox_->setSuffix(" N");
    force_spinbox_->setDecimals(1);
    force_spinbox_->setValue(0.0);
    force_layout->addWidget(force_slider_);
    force_layout->addWidget(force_spinbox_);

    // Velocity control
    auto velocity_layout = new QHBoxLayout();
    velocity_layout->addWidget(new QLabel("Velocity (rad/s):"));
    velocity_slider_ = new QSlider(Qt::Horizontal);
    velocity_slider_->setRange(0, 785);
    velocity_slider_->setValue(0);
    velocity_spinbox_ = new QDoubleSpinBox();
    velocity_spinbox_->setRange(0.0, 785.0);
    velocity_spinbox_->setSuffix(" rad/s");
    velocity_spinbox_->setValue(0.0);
    velocity_layout->addWidget(velocity_slider_);
    velocity_layout->addWidget(velocity_spinbox_);

    // Control buttons
    auto button_layout = new QHBoxLayout();
    stop_btn_ = new QPushButton("EMERGENCY STOP");
    stop_btn_->setObjectName("stopButton");
    stabilize_btn_ = new QPushButton("Stabilize");
    refresh_btn_ = new QPushButton("Refresh");

    button_layout->addWidget(stop_btn_);
    button_layout->addWidget(stabilize_btn_);
    button_layout->addWidget(refresh_btn_);

    // Add to layout
    layout->addLayout(angle_layout);
    layout->addLayout(force_layout);
    layout->addLayout(velocity_layout);
    layout->addLayout(button_layout);
    layout->addStretch();

    // Connect signals
    connect(angle_slider_, &QSlider::valueChanged, this, &MainWindow::onAngleSliderChanged);
    connect(force_slider_, &QSlider::valueChanged, this, &MainWindow::onForceSliderChanged);
    connect(velocity_slider_, &QSlider::valueChanged, this, &MainWindow::onVelocitySliderChanged);
    connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(stabilize_btn_, &QPushButton::clicked, this, &MainWindow::onStabilizeClicked);
    connect(refresh_btn_, &QPushButton::clicked, this, &MainWindow::onRefreshClicked);

    // Sync spinboxes with sliders
    connect(angle_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value)
            { angle_slider_->setValue(static_cast<int>(value)); });
    connect(force_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value)
            { force_slider_->setValue(static_cast<int>(value * 10)); });
    connect(velocity_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            [this](double value)
            { velocity_slider_->setValue(static_cast<int>(value)); });
}

void MainWindow::setupMonitoringPanel()
{
    monitor_group_ = new QGroupBox("System Monitoring");
    auto layout = new QVBoxLayout(monitor_group_);

    // Create value labels
    auto createValueRow = [this](const QString &label, QLabel *&value_label)
    {
        auto row = new QHBoxLayout();
        auto label_widget = new QLabel(label + ":");
        label_widget->setMinimumWidth(120);
        value_label = new QLabel("0.00");
        value_label->setObjectName("valueLabel");
        value_label->setAlignment(Qt::AlignCenter);
        value_label->setMinimumWidth(100);
        row->addWidget(label_widget);
        row->addWidget(value_label);
        row->addStretch();
        return row;
    };

    layout->addLayout(createValueRow("Arm Angle", arm_angle_value_));
    layout->addLayout(createValueRow("Motor Speed", motor_speed_value_));
    layout->addLayout(createValueRow("V EMF", v_emf_value_));
    layout->addLayout(createValueRow("ΔV EMF", delta_v_emf_value_));
    layout->addLayout(createValueRow("Error", error_value_));
    layout->addLayout(createValueRow("Motor Cmd", motor_cmd_value_));

    // Progress bars
    auto thrust_row = new QHBoxLayout();
    thrust_row->addWidget(new QLabel("Thrust:"));
    thrust_progress_ = new QProgressBar();
    thrust_progress_->setRange(0, 450); // 0-45.0 N
    thrust_progress_->setFormat("%v/45.0 N");
    thrust_row->addWidget(thrust_progress_);

    auto angle_row = new QHBoxLayout();
    angle_row->addWidget(new QLabel("Angle:"));
    angle_progress_ = new QProgressBar();
    angle_progress_->setRange(-90, 90);
    angle_progress_->setFormat("%v°");
    angle_row->addWidget(angle_progress_);

    layout->addLayout(thrust_row);
    layout->addLayout(angle_row);
    layout->addStretch();
}

void MainWindow::setupPlotsPanel()
{
    plots_group_ = new QGroupBox("Real-Time Data Plots");
    auto layout = new QHBoxLayout(plots_group_);

    // Create plots
    angle_plot_ = new RealTimePlot("Arm Angle", "Angle", "degrees");
    angle_plot_->setLineColor(QColor(ACCENT_COLOR));
    angle_plot_->setYRange(-90, 90);
    angle_plot_->setTimeWindow(30); // 30 seconds

    error_plot_ = new RealTimePlot("Control Error", "Error", "degrees");
    error_plot_->setLineColor(QColor(WARNING_COLOR));
    error_plot_->setAutoScale(true);
    error_plot_->setTimeWindow(30);

    motor_plot_ = new RealTimePlot("Motor Command", "Speed", "rad/s");
    motor_plot_->setLineColor(QColor(SUCCESS_COLOR));
    motor_plot_->setYRange(0, 785);
    motor_plot_->setTimeWindow(30);

    layout->addWidget(angle_plot_);
    layout->addWidget(error_plot_);
    layout->addWidget(motor_plot_);
}

void MainWindow::createStatusBar()
{
    auto status_bar = statusBar();

    connection_status_ = new QLabel("DISCONNECTED");
    connection_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));

    control_mode_ = new QLabel("Manual");
    control_mode_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(ACCENT_COLOR));

    system_status_ = new QLabel("Ready");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));

    status_bar->addWidget(new QLabel("Connection:"));
    status_bar->addWidget(connection_status_);
    status_bar->addPermanentWidget(new QLabel("Mode:"));
    status_bar->addPermanentWidget(control_mode_);
    status_bar->addPermanentWidget(new QLabel("Status:"));
    status_bar->addPermanentWidget(system_status_);
}

void MainWindow::updateDisplays()
{
    if (!ros_node_)
        return;

    auto data = ros_node_->getCurrentData();
    if (!data.valid)
        return;

    // Update value labels
    arm_angle_value_->setText(QString::number(data.arm_angle_deg, 'f', 2) + "°");
    motor_speed_value_->setText(QString::number(data.motor_speed_est, 'f', 1) + " rad/s");
    v_emf_value_->setText(QString::number(data.v_emf, 'f', 3) + " V");
    delta_v_emf_value_->setText(QString::number(data.delta_v_emf, 'f', 3) + " V");
    error_value_->setText(QString::number(data.error, 'f', 2) + "°");
    motor_cmd_value_->setText(QString::number(data.motor_command, 'f', 1) + " rad/s");

    // Update progress bars
    thrust_progress_->setValue(static_cast<int>(data.motor_command * 0.57)); // Approximate thrust conversion
    angle_progress_->setValue(static_cast<int>(data.arm_angle_deg));

    // Update dashboard
    dashboard_->setArmAngle(data.arm_angle_deg);
    dashboard_->setTargetAngle(data.target_angle);
    dashboard_->setMotorSpeed(data.motor_speed_est);
    dashboard_->setThrust(data.motor_command * 0.057); // Approximate
    dashboard_->setError(data.error);
    dashboard_->setVEmf(data.v_emf);
    dashboard_->setConnectionStatus(ros_node_->isConnected());
    dashboard_->setSystemStatus(QString::fromStdString(ros_node_->getControlMode()));

    // Update plots
    double time_sec = data.timestamp.seconds() + data.timestamp.nanoseconds() * 1e-9;
    angle_plot_->addDataPoint(data.arm_angle_deg, time_sec);
    error_plot_->addDataPoint(data.error, time_sec);
    motor_plot_->addDataPoint(data.motor_command, time_sec);

    // Update status
    control_mode_->setText(QString::fromStdString(ros_node_->getControlMode()));
    system_status_->setText(data.valid ? "Active" : "Inactive");
}

void MainWindow::onAngleSliderChanged(int value)
{
    angle_spinbox_->setValue(value);
    ros_node_->sendAngleCommand(value);
    control_mode_->setText("Angle Control");
}

void MainWindow::onForceSliderChanged(int value)
{
    double force = value / 10.0; // Convert back to actual force
    force_spinbox_->setValue(force);
    ros_node_->sendForceCommand(force);
    control_mode_->setText("Force Control");
}

void MainWindow::onVelocitySliderChanged(int value)
{
    velocity_spinbox_->setValue(value);
    ros_node_->sendVelocityCommand(value);
    control_mode_->setText("Velocity Control");
}

void MainWindow::onStopClicked()
{
    ros_node_->sendStopCommand();

    // Reset all controls
    angle_slider_->setValue(0);
    force_slider_->setValue(0);
    velocity_slider_->setValue(0);
    angle_spinbox_->setValue(0);
    force_spinbox_->setValue(0);
    velocity_spinbox_->setValue(0);

    control_mode_->setText("STOPPED");
    system_status_->setText("Emergency Stop");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));
}

void MainWindow::onStabilizeClicked()
{
    ros_node_->sendAngleCommand(0.0); // Stabilize at horizontal
    angle_slider_->setValue(0);
    angle_spinbox_->setValue(0);
    control_mode_->setText("Stabilizing");
}

void MainWindow::onRefreshClicked()
{
    // Clear plots and reset displays
    angle_plot_->clearData();
    error_plot_->clearData();
    motor_plot_->clearData();

    system_status_->setText("Ready");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));
}
#include "prop_arm_gui/main_window.hpp"
#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include <QApplication>
#include <QScreen>

MainWindow::MainWindow(std::shared_ptr<PropArmGuiNode> node, QWidget *parent)
    : QMainWindow(parent), ros_node_(node)
{
    setupUbuntuScreenGeometry();
    setupUI();
    setupStyles();

    // Create update timer for real-time display updates
    update_timer_ = new QTimer(this);
    update_timer_->setInterval(50); // 20 Hz update rate
    connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateDisplays);
    update_timer_->start();

    // Connect ROS node signals
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
    }

    setWindowTitle("PropArm Aerospace Control System - Enhanced Voltage Monitoring");
}

void MainWindow::setupUI()
{
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);

    // Create main tab widget
    tab_widget_ = new QTabWidget(central_widget_);

    // Create visualization tab (6 charts in 2x3 layout)
    data_visualizer_ = new AerospaceDataVisualizer(ros_node_, this);
    tab_widget_->addTab(data_visualizer_, "Enhanced Telemetry Visualization");

    // Create control tab
    setupControlTab();
    tab_widget_->addTab(control_widget_, "System Control");

    // Main layout
    QVBoxLayout *main_layout = new QVBoxLayout(central_widget_);
    main_layout->addWidget(tab_widget_);
    main_layout->setContentsMargins(8, 8, 8, 8);

    // Create status bar
    createStatusBar();
}

void MainWindow::setupControlTab()
{
    control_widget_ = new QWidget();
    QHBoxLayout *control_layout = new QHBoxLayout(control_widget_);
    control_layout->setSpacing(12);
    control_layout->setContentsMargins(12, 12, 12, 12);

    // Setup control and monitoring panels
    setupControlPanel();
    setupMonitoringPanel();

    control_layout->addWidget(control_group_);
    control_layout->addWidget(monitor_group_);
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

    // Control Buttons
    QHBoxLayout *button_layout = new QHBoxLayout();

    stop_btn_ = new QPushButton("STOP");
    stop_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(DANGER_COLOR));

    stabilize_btn_ = new QPushButton("STABILIZE");
    stabilize_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(SUCCESS_COLOR));

    refresh_btn_ = new QPushButton("REFRESH");
    refresh_btn_->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; padding: 8px; }").arg(SECONDARY_COLOR));

    button_layout->addWidget(stop_btn_);
    button_layout->addWidget(stabilize_btn_);
    button_layout->addWidget(refresh_btn_);

    // Add to control group
    control_group_layout->addWidget(angle_group);
    control_group_layout->addWidget(velocity_group);
    control_group_layout->addLayout(button_layout);
    control_group_layout->addStretch();

    // Connect signals
    connect(angle_slider_, &QSlider::valueChanged, this, &MainWindow::onAngleSliderChanged);
    connect(velocity_slider_, &QSlider::valueChanged, this, &MainWindow::onVelocitySliderChanged);
    connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(stabilize_btn_, &QPushButton::clicked, this, &MainWindow::onStabilizeClicked);
    connect(refresh_btn_, &QPushButton::clicked, this, &MainWindow::onRefreshClicked);

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
    monitor_group_ = new QGroupBox("Enhanced System Monitoring");
    QGridLayout *monitor_layout = new QGridLayout(monitor_group_);

    // Enhanced monitoring labels with voltage focus
    QStringList labels = {
        "Current Angle:", "Motor Speed:", "Control Error:",
        "Vref (Input):", "V_EMF:", "VPWM:", "Motor Command:"};

    // Create value labels with enhanced styling
    arm_angle_value_ = new QLabel("0.0 °");
    motor_speed_value_ = new QLabel("0.0 rad/s");
    error_value_ = new QLabel("0.0 °");
    vref_value_ = new QLabel("0.0 V");
    QLabel *v_emf_label = new QLabel("V_EMF:");
    v_emf_value_ = new QLabel("0.0 V");
    vpwm_value_ = new QLabel("0.0 V");
    motor_cmd_value_ = new QLabel("0.0");

    // Progress bars for visual feedback
    angle_progress_ = new QProgressBar();
    angle_progress_->setRange(-90, 90);
    angle_progress_->setValue(0);
    angle_progress_->setTextVisible(false);
    angle_progress_->setStyleSheet(QString("QProgressBar::chunk { background-color: %1; }").arg(PRIMARY_COLOR));

    velocity_progress_ = new QProgressBar();
    velocity_progress_->setRange(-785, 785);
    velocity_progress_->setValue(0);
    velocity_progress_->setTextVisible(false);
    velocity_progress_->setStyleSheet(QString("QProgressBar::chunk { background-color: %1; }").arg(SUCCESS_COLOR));

    // Layout arrangement
    int row = 0;
    monitor_layout->addWidget(new QLabel("Current Angle:"), row, 0);
    monitor_layout->addWidget(arm_angle_value_, row, 1);
    monitor_layout->addWidget(angle_progress_, row++, 2);

    monitor_layout->addWidget(new QLabel("Motor Speed:"), row, 0);
    monitor_layout->addWidget(motor_speed_value_, row, 1);
    monitor_layout->addWidget(velocity_progress_, row++, 2);

    monitor_layout->addWidget(new QLabel("Control Error:"), row, 0);
    monitor_layout->addWidget(error_value_, row++, 1);

    monitor_layout->addWidget(new QLabel("Vref (Input):"), row, 0);
    monitor_layout->addWidget(vref_value_, row++, 1);

    monitor_layout->addWidget(v_emf_label, row, 0);
    monitor_layout->addWidget(v_emf_value_, row++, 1);

    monitor_layout->addWidget(new QLabel("VPWM:"), row, 0);
    monitor_layout->addWidget(vpwm_value_, row++, 1);

    monitor_layout->addWidget(new QLabel("Motor Command:"), row, 0);
    monitor_layout->addWidget(motor_cmd_value_, row++, 1);

    // Apply enhanced styling to value labels
    for (auto *label : {arm_angle_value_, motor_speed_value_, error_value_,
                        vref_value_, v_emf_value_, vpwm_value_, motor_cmd_value_})
    {
        label->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12px;").arg(ACCENT_COLOR));
        label->setAlignment(Qt::AlignRight);
    }
}

void MainWindow::createStatusBar()
{
    // Enhanced status bar with voltage monitoring indicators
    QWidget *status_widget = new QWidget();
    QHBoxLayout *status_layout = new QHBoxLayout(status_widget);
    status_layout->setContentsMargins(8, 4, 8, 4);

    connection_status_ = new QLabel("Disconnected");
    connection_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(DANGER_COLOR));

    control_mode_ = new QLabel("Manual");
    control_mode_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(TEXT_COLOR));

    system_status_ = new QLabel("Voltage Monitoring Active");
    system_status_->setStyleSheet(QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR));

    status_layout->addWidget(new QLabel("Connection:"));
    status_layout->addWidget(connection_status_);
    status_layout->addWidget(new QLabel("|"));
    status_layout->addWidget(new QLabel("Mode:"));
    status_layout->addWidget(control_mode_);
    status_layout->addWidget(new QLabel("|"));
    status_layout->addWidget(new QLabel("System:"));
    status_layout->addWidget(system_status_);
    status_layout->addStretch();

    statusBar()->addPermanentWidget(status_widget);
}

void MainWindow::setupStyles()
{
    setStyleSheet(QString(R"(
        QMainWindow {
            background-color: %1;
            color: %2;
        }
        QTabWidget::pane {
            border: 1px solid %3;
            background-color: %1;
        }
        QTabWidget::tab-bar {
            alignment: center;
        }
        QTabBar::tab {
            background-color: %4;
            color: %2;
            padding: 8px 16px;
            margin-right: 2px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }
        QTabBar::tab:selected {
            background-color: %5;
            color: white;
        }
        QGroupBox {
            border: 2px solid %3;
            border-radius: 8px;
            margin-top: 8px;
            padding-top: 8px;
            font-weight: bold;
            color: %6;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 8px 0 8px;
        }
        QSlider::groove:horizontal {
            border: 1px solid %3;
            height: 8px;
            background: %4;
            border-radius: 4px;
        }
        QSlider::handle:horizontal {
            background: %6;
            border: 1px solid %3;
            width: 18px;
            margin: -5px 0;
            border-radius: 9px;
        }
        QSpinBox, QDoubleSpinBox {
            background-color: %4;
            border: 1px solid %3;
            border-radius: 4px;
            padding: 4px;
            color: %2;
        }
        QPushButton {
            border: 1px solid %3;
            border-radius: 6px;
            padding: 6px 12px;
            font-weight: bold;
        }
        QPushButton:hover {
            border: 2px solid %6;
        }
        QPushButton:pressed {
            background-color: %3;
        }
        QProgressBar {
            border: 1px solid %3;
            border-radius: 4px;
            background-color: %4;
            text-align: center;
        }
        QStatusBar {
            background-color: %4;
            border-top: 1px solid %3;
        }
    )")
                      .arg(BACKGROUND_COLOR)
                      .arg(TEXT_COLOR)
                      .arg(PRIMARY_COLOR)
                      .arg(CARD_COLOR)
                      .arg(SECONDARY_COLOR)
                      .arg(ACCENT_COLOR));
}

void MainWindow::setupUbuntuScreenGeometry()
{
    // Enhanced screen geometry setup for Ubuntu with Qt6
    QScreen *screen = QGuiApplication::primaryScreen();
    if (screen)
    {
        QRect screen_geometry = screen->availableGeometry();

        // Calculate window size (90% of available screen)
        int window_width = static_cast<int>(screen_geometry.width() * 0.9);
        int window_height = static_cast<int>(screen_geometry.height() * 0.9);

        // Center the window
        int x = screen_geometry.x() + (screen_geometry.width() - window_width) / 2;
        int y = screen_geometry.y() + (screen_geometry.height() - window_height) / 2;

        setGeometry(x, y, window_width, window_height);

        // Set minimum size for aerospace control interface
        setMinimumSize(1400, 900);
    }
    else
    {
        // Fallback sizing
        setGeometry(100, 100, 1600, 1000);
        setMinimumSize(1400, 900);
    }
}

void MainWindow::updateDisplays()
{
    if (!ros_node_)
        return;

    auto data = ros_node_->getCurrentData();

    // Update monitoring values with enhanced voltage display
    arm_angle_value_->setText(QString::number(data.arm_angle_deg, 'f', 2) + " °");
    motor_speed_value_->setText(QString::number(data.motor_speed_est, 'f', 1) + " rad/s");
    error_value_->setText(QString::number(data.error, 'f', 2) + " °");

    // Enhanced voltage monitoring
    vref_value_->setText(QString::number(data.vref_input, 'f', 3) + " V");
    v_emf_value_->setText(QString::number(data.v_emf, 'f', 3) + " V");
    vpwm_value_->setText(QString::number(data.vpwm, 'f', 3) + " V");
    motor_cmd_value_->setText(QString::number(data.motor_command, 'f', 2));

    // Update progress bars
    angle_progress_->setValue(static_cast<int>(data.arm_angle_deg));
    velocity_progress_->setValue(static_cast<int>(data.motor_speed_est));

    // Update control mode and connection status
    control_mode_->setText(QString::fromStdString(ros_node_->getControlMode()));

    // Update data visualizer with enhanced voltage data
    if (data_visualizer_)
    {
        data_visualizer_->onDataReceived(
            data.arm_angle_deg,
            data.motor_speed_est,
            data.v_emf,
            data.error,
            data.target_angle,
            data.vpwm,
            data.vref_input);
    }

    // Color coding for voltage levels
    auto updateVoltageColor = [](QLabel *label, double voltage, double nominal)
    {
        QString color;
        if (voltage < nominal * 0.1)
            color = "#ff3366"; // Red for very low
        else if (voltage < nominal * 0.5)
            color = "#ff8c00"; // Orange for low
        else if (voltage > nominal * 1.1)
            color = "#ff8c00"; // Orange for high
        else
            color = "#00ff88"; // Green for normal

        label->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12px;").arg(color));
    };

    updateVoltageColor(vref_value_, data.vref_input, 11.0);
    updateVoltageColor(v_emf_value_, data.v_emf, 11.0);
    updateVoltageColor(vpwm_value_, data.vpwm, 11.0);
}

void MainWindow::onAngleSliderChanged(int value)
{
    angle_spinbox_->setValue(value);
    if (ros_node_)
    {
        ros_node_->sendAngleCommand(value);
    }
}

void MainWindow::onVelocitySliderChanged(int value)
{
    velocity_spinbox_->setValue(value);
    if (ros_node_)
    {
        ros_node_->sendVelocityCommand(value);
    }
}

void MainWindow::onStopClicked()
{
    if (ros_node_)
    {
        ros_node_->sendStopCommand();
    }

    // Reset sliders to zero
    angle_slider_->setValue(0);
    velocity_slider_->setValue(0);
    angle_spinbox_->setValue(0.0);
    velocity_spinbox_->setValue(0.0);
}

void MainWindow::onStabilizeClicked()
{
    if (ros_node_)
    {
        auto data = ros_node_->getCurrentData();
        // Set target to current angle for stabilization
        angle_slider_->setValue(static_cast<int>(data.arm_angle_deg));
        ros_node_->sendAngleCommand(data.arm_angle_deg);
    }
}

void MainWindow::onRefreshClicked()
{
    if (data_visualizer_)
    {
        data_visualizer_->clearData();
    }
}

void MainWindow::applyConfigToControls()
{
    // Apply any configuration updates to controls
    if (data_visualizer_)
    {
        data_visualizer_->updateTheme();
    }
}

#include "main_window.moc"
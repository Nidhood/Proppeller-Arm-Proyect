#include "prop_arm_gui/main_window.hpp"
#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include <QApplication>
#include <QStatusBar>
#include <QMenuBar>
#include <QSplitter>
#include <QTabWidget>
#include <QMessageBox>
#include <QScreen>

MainWindow::MainWindow(std::shared_ptr<PropArmGuiNode> node, QWidget *parent)
    : QMainWindow(parent), ros_node_(node)
{
    setWindowTitle("PropArm Control System - Aerospace Data Visualization");

    // FIXED: Updated screen geometry setup for Qt6
    setupUbuntuScreenGeometry();

    setupStyles();
    setupUI();

    // Connect ROS node signals with thread-safe connections
    connect(ros_node_.get(), &PropArmGuiNode::dataUpdated,
            this, &MainWindow::updateDisplays, Qt::QueuedConnection);
    connect(ros_node_.get(), &PropArmGuiNode::connectionChanged, this, [this](bool connected)
            { QMetaObject::invokeMethod(this, [this, connected]()
                                        {
                    connection_status_->setText(connected ? "CONNECTED" : "DISCONNECTED");
                    connection_status_->setStyleSheet(connected ? 
                        QString("color: %1; font-weight: bold;").arg(SUCCESS_COLOR) :
                        QString("color: %1; font-weight: bold;").arg(DANGER_COLOR)); }, Qt::QueuedConnection); }, Qt::QueuedConnection);

    // Setup update timer with reduced frequency to prevent UI freezing
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateDisplays);
    update_timer_->start(200);

    // Initial update
    updateDisplays();
}

void MainWindow::setupUbuntuScreenGeometry()
{
    // Get the primary screen (Qt6 approach)
    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen)
    {
        // Fallback to first available screen
        auto screens = QGuiApplication::screens();
        if (!screens.isEmpty())
        {
            screen = screens.first();
        }
        else
        {
            // Last resort fallback - use window defaults
            setMinimumSize(1400, 900);
            resize(1600, 1000);
            return;
        }
    }

    // Get screen geometries
    QRect screenGeometry = screen->geometry();
    QRect availableGeometry = screen->availableGeometry();

    // Fixed: Remove unused variables and calculate window size directly
    // Set window geometry with proper margins for Ubuntu
    int windowWidth = availableGeometry.width() - 40;   // 20px margin on each side
    int windowHeight = availableGeometry.height() - 40; // 20px margin top/bottom

    // Position window in available space
    int x = availableGeometry.x() + 20;
    int y = availableGeometry.y() + 20;

    // Set minimum size
    setMinimumSize(1400, 900);

    // Apply geometry
    setGeometry(x, y, windowWidth, windowHeight);

    // Log screen info for debugging
    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Screen setup - Available: %dx%d at (%d,%d), Window: %dx%d at (%d,%d)",
                    availableGeometry.width(), availableGeometry.height(),
                    availableGeometry.x(), availableGeometry.y(),
                    windowWidth, windowHeight, x, y);
    }
}

void MainWindow::setupUI()
{
    central_widget_ = new QWidget;
    setCentralWidget(central_widget_);

    // Create main tabbed interface
    tab_widget_ = new QTabWidget();
    tab_widget_->setTabPosition(QTabWidget::North);

    // Tab 1: Data Visualization (Main focus)
    data_visualizer_ = new AerospaceDataVisualizer(ros_node_);
    tab_widget_->addTab(data_visualizer_, "🚀 Data Visualization");

    // Tab 2: Control Panel (Secondary)
    control_widget_ = new QWidget();
    setupControlTab();
    tab_widget_->addTab(control_widget_, "⚙️ Control Panel");

    // Main layout
    auto main_layout = new QVBoxLayout(central_widget_);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->addWidget(tab_widget_);

    createStatusBar();

    // Style the tab widget
    tab_widget_->setStyleSheet(QString(R"(
        QTabWidget::pane {
            border: 2px solid %1;
            background-color: %2;
            border-radius: 8px;
        }
        QTabBar::tab {
            background: %3;
            color: %4;
            padding: 12px 20px;
            margin-right: 2px;
            border-top-left-radius: 8px;
            border-top-right-radius: 8px;
            font-size: 12pt;
            font-weight: bold;
        }
        QTabBar::tab:selected {
            background: %1;
            color: %2;
        }
        QTabBar::tab:hover {
            background: %5;
        }
    )")
                                   .arg(ACCENT_COLOR, BACKGROUND_COLOR, CARD_COLOR, TEXT_COLOR, SECONDARY_COLOR));
}

void MainWindow::setupMonitoringPanel()
{
    monitor_group_ = new QGroupBox("📡 TELEMETRY MONITOR");
    auto layout = new QVBoxLayout(monitor_group_);
    layout->setSpacing(12);

    // Create enhanced value display function
    auto createValueDisplay = [this](const QString &label, const QString &units, QLabel *&value_label)
    {
        auto frame = new QFrame();
        frame->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
        frame->setStyleSheet(QString(R"(
            QFrame {
                background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1,
                    stop: 0 %1, stop: 1 %2);
                border: 1px solid %3;
                border-radius: 8px;
                padding: 5px;
            }
        )")
                                 .arg(CARD_COLOR, BACKGROUND_COLOR, PRIMARY_COLOR));

        auto frame_layout = new QVBoxLayout(frame);
        frame_layout->setContentsMargins(10, 8, 10, 8);

        auto label_widget = new QLabel(label);
        label_widget->setAlignment(Qt::AlignCenter);
        label_widget->setStyleSheet("font-size: 10pt; font-weight: bold; color: " + TEXT_COLOR + ";");

        value_label = new QLabel("-- " + units);
        value_label->setObjectName("valueLabel");
        value_label->setAlignment(Qt::AlignCenter);
        value_label->setMinimumHeight(35);

        frame_layout->addWidget(label_widget);
        frame_layout->addWidget(value_label);

        return frame;
    };

    // Create monitoring displays
    layout->addWidget(createValueDisplay("ARM ANGLE", "°", arm_angle_value_));
    layout->addWidget(createValueDisplay("MOTOR SPEED", "rad/s", motor_speed_value_));
    layout->addWidget(createValueDisplay("V EMF", "V", v_emf_value_));
    layout->addWidget(createValueDisplay("DELTA V EMF", "V", delta_v_emf_value_));
    layout->addWidget(createValueDisplay("CONTROL ERROR", "°", error_value_));
    layout->addWidget(createValueDisplay("MOTOR CMD", "rad/s", motor_cmd_value_));

    // Progress indicators
    auto progress_frame = new QFrame();
    progress_frame->setFrameStyle(QFrame::StyledPanel);
    auto progress_layout = new QVBoxLayout(progress_frame);

    auto angle_progress_layout = new QHBoxLayout();
    auto angle_progress_label = new QLabel("ANGLE POSITION:");
    angle_progress_label->setMinimumWidth(120);
    angle_progress_ = new QProgressBar();
    angle_progress_->setRange(-90, 90);
    angle_progress_->setFormat("%v°");
    angle_progress_->setMinimumHeight(25);
    angle_progress_layout->addWidget(angle_progress_label);
    angle_progress_layout->addWidget(angle_progress_);

    auto velocity_progress_layout = new QHBoxLayout();
    auto velocity_progress_label = new QLabel("MOTOR VELOCITY:");
    velocity_progress_label->setMinimumWidth(120);
    velocity_progress_ = new QProgressBar();
    velocity_progress_->setRange(0, 785);
    velocity_progress_->setFormat("%v rad/s");
    velocity_progress_->setMinimumHeight(25);
    velocity_progress_layout->addWidget(velocity_progress_label);
    velocity_progress_layout->addWidget(velocity_progress_);

    progress_layout->addLayout(angle_progress_layout);
    progress_layout->addLayout(velocity_progress_layout);

    layout->addWidget(progress_frame);
    layout->addStretch();
}

void MainWindow::createStatusBar()
{
    auto status_bar = statusBar();
    status_bar->setMinimumHeight(35);

    // Enhanced status indicators
    auto createStatusLabel = [](const QString &text, const QString &color)
    {
        auto label = new QLabel(text);
        label->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12pt; padding: 5px 10px; background-color: rgba(255,255,255,0.1); border-radius: 4px;").arg(color));
        return label;
    };

    connection_status_ = createStatusLabel("DISCONNECTED", DANGER_COLOR);
    control_mode_ = createStatusLabel("Manual", ACCENT_COLOR);
    system_status_ = createStatusLabel("Ready", SUCCESS_COLOR);

    status_bar->addWidget(new QLabel("🔗 Connection:"));
    status_bar->addWidget(connection_status_);
    status_bar->addPermanentWidget(new QLabel("🎮 Mode:"));
    status_bar->addPermanentWidget(control_mode_);
    status_bar->addPermanentWidget(new QLabel("⚡ Status:"));
    status_bar->addPermanentWidget(system_status_);
}

void MainWindow::updateDisplays()
{
    if (!ros_node_)
        return;

    auto data = ros_node_->getCurrentData();
    if (!data.valid)
        return;

    // Use thread-safe updates for UI elements
    QMetaObject::invokeMethod(this, [this, data]()
                              {
        // Update value labels with enhanced formatting
        arm_angle_value_->setText(QString::number(data.arm_angle_deg, 'f', 2) + "°");
        motor_speed_value_->setText(QString::number(data.motor_speed_est, 'f', 1) + " rad/s");
        v_emf_value_->setText(QString::number(data.v_emf, 'f', 3) + " V");
        delta_v_emf_value_->setText(QString::number(data.delta_v_emf, 'f', 3) + " V");
        error_value_->setText(QString::number(data.error, 'f', 2) + "°");
        motor_cmd_value_->setText(QString::number(data.motor_command, 'f', 1) + " rad/s");

        // Update progress bars
        angle_progress_->setValue(static_cast<int>(data.arm_angle_deg));
        velocity_progress_->setValue(static_cast<int>(std::abs(data.motor_speed_est)));

        // Update status
        control_mode_->setText(QString::fromStdString(ros_node_->getControlMode()));
        system_status_->setText(data.valid ? "ACTIVE" : "INACTIVE");

        // Color coding for status
        QString status_color = data.valid ? SUCCESS_COLOR : WARNING_COLOR;
        system_status_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12pt; padding: 5px 10px; background-color: rgba(255,255,255,0.1); border-radius: 4px;").arg(status_color)); }, Qt::QueuedConnection);

    // Update data visualizer (main focus)
    if (data_visualizer_)
    {
        data_visualizer_->onDataReceived(
            data.arm_angle_deg,
            data.motor_speed_est,
            data.v_emf,
            data.delta_v_emf,
            data.error,
            data.target_angle,
            data.motor_command);
    }
}

void MainWindow::onAngleSliderChanged(int value)
{
    angle_spinbox_->setValue(value);
    ros_node_->sendAngleCommand(value);
    QMetaObject::invokeMethod(this, [this]()
                              { control_mode_->setText("ANGLE CONTROL"); }, Qt::QueuedConnection);
}

void MainWindow::onVelocitySliderChanged(int value)
{
    velocity_spinbox_->setValue(value);
    ros_node_->sendVelocityCommand(value);
    QMetaObject::invokeMethod(this, [this]()
                              { control_mode_->setText("VELOCITY CONTROL"); }, Qt::QueuedConnection);
}

void MainWindow::onStopClicked()
{
    ros_node_->sendStopCommand();

    // Reset all controls with animation effect
    QMetaObject::invokeMethod(this, [this]()
                              {
        angle_slider_->setValue(0);
        velocity_slider_->setValue(0);
    velocity_slider_->setTracking(false);  // Publish only on release to avoid noise
        angle_spinbox_->setValue(0);
        velocity_spinbox_->setValue(0);

        control_mode_->setText("🛑 EMERGENCY STOP");
        control_mode_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12pt; padding: 5px 10px; background-color: rgba(255,0,0,0.2); border-radius: 4px;").arg(DANGER_COLOR));

        system_status_->setText("EMERGENCY STOP ACTIVE");
        system_status_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12pt; padding: 5px 10px; background-color: rgba(255,0,0,0.2); border-radius: 4px;").arg(DANGER_COLOR)); }, Qt::QueuedConnection);
}

void MainWindow::onStabilizeClicked()
{
    ros_node_->sendAngleCommand(0.0);
    QMetaObject::invokeMethod(this, [this]()
                              {
        angle_slider_->setValue(0);
        angle_spinbox_->setValue(0);
        control_mode_->setText("⚖️ STABILIZING");
        control_mode_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12pt; padding: 5px 10px; background-color: rgba(0,255,200,0.2); border-radius: 4px;").arg(ACCENT_COLOR)); }, Qt::QueuedConnection);
}

void MainWindow::onRefreshClicked()
{
    // Clear all visualizations
    if (data_visualizer_)
    {
        data_visualizer_->clearData();
    }

    // Reset status
    QMetaObject::invokeMethod(this, [this]()
                              {
        system_status_->setText("READY");
        system_status_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12pt; padding: 5px 10px; background-color: rgba(255,255,255,0.1); border-radius: 4px;").arg(SUCCESS_COLOR));

        control_mode_->setText("MANUAL");
        control_mode_->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 12pt; padding: 5px 10px; background-color: rgba(255,255,255,0.1); border-radius: 4px;").arg(ACCENT_COLOR)); }, Qt::QueuedConnection);
}

void MainWindow::setupControlTab()
{
    auto layout = new QGridLayout(control_widget_);
    layout->setSpacing(15);
    layout->setContentsMargins(20, 20, 20, 20);

    // Create control groups
    setupControlPanel();
    setupMonitoringPanel();

    // Add to layout
    layout->addWidget(control_group_, 0, 0);
    layout->addWidget(monitor_group_, 0, 1);
    layout->setRowStretch(1, 1);
}

void MainWindow::setupStyles()
{
    // Enhanced aerospace theme
    QString main_style = QString(R"(
        QMainWindow {
            background-color: %1;
            color: %2;
            font-family: 'Consolas', 'Monaco', monospace;
        }
        QWidget {
            background-color: %1;
            color: %2;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid %3;
            border-radius: 12px;
            margin-top: 1ex;
            padding-top: 15px;
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                stop: 0 %4, stop: 1 %1);
            color: %2;
            font-size: 13pt;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 15px;
            padding: 0 10px 0 10px;
            color: %5;
            font-size: 14pt;
            font-weight: bold;
        }
        QPushButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                stop: 0 %3, stop: 1 %6);
            border: 2px solid %5;
            border-radius: 8px;
            padding: 12px 20px;
            font-weight: bold;
            color: %2;
            min-width: 100px;
            font-size: 11pt;
        }
        QPushButton:hover {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                stop: 0 %5, stop: 1 %3);
            color: %1;
            border: 3px solid %5;
        }
        QPushButton:pressed {
            background: %6;
            border-color: %2;
        }
        QPushButton#stopButton {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                stop: 0 %7, stop: 1 #8b1538);
            border-color: %7;
        }
        QPushButton#stopButton:hover {
            background: #dc2626;
            border: 3px solid %5;
        }
        QSlider::groove:horizontal {
            border: 2px solid %3;
            height: 10px;
            background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                stop: 0 %1, stop: 1 %4);
            border-radius: 6px;
        }
        QSlider::handle:horizontal {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                fx: 0.3, fy: 0.3, stop: 0 %5, stop: 1 %3);
            border: 2px solid %2;
            width: 24px;
            height: 24px;
            border-radius: 14px;
            margin: -8px 0;
            border: 3px solid %5;
        }
        QSlider::handle:horizontal:hover {
            background: qradialgradient(cx: 0.5, cy: 0.5, radius: 0.5,
                fx: 0.3, fy: 0.3, stop: 0 %2, stop: 1 %5);
            border: 3px solid %5;
        }
        QProgressBar {
            border: 2px solid %3;
            border-radius: 8px;
            background-color: %1;
            text-align: center;
            font-weight: bold;
            color: %2;
            font-size: 11pt;
        }
        QProgressBar::chunk {
            background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                stop: 0 %5, stop: 0.5 %6, stop: 1 %5);
            border-radius: 6px;
            
        }
        QLabel {
            color: %2;
            font-size: 12pt;
        }
        QLabel#valueLabel {
            font-size: 16pt;
            font-weight: bold;
            color: %5;
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                stop: 0 %1, stop: 1 %4);
            border: 2px solid %3;
            border-radius: 6px;
            padding: 8px 12px;
            font-family: 'Courier New', monospace;
        }
        QSpinBox, QDoubleSpinBox {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                stop: 0 %1, stop: 1 %4);
            border: 2px solid %3;
            border-radius: 6px;
            padding: 6px;
            color: %2;
            font-weight: bold;
            font-size: 11pt;
        }
        QSpinBox:focus, QDoubleSpinBox:focus {
            border-color: %5;
            border: 3px solid %5;
        }
        QStatusBar {
            background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                stop: 0 %4, stop: 1 %1);
            border-top: 1px solid %3;
            color: %2;
            font-size: 11pt;
        }
        QStatusBar::item {
            border: none;
        }
    )")
                             .arg(BACKGROUND_COLOR, TEXT_COLOR, PRIMARY_COLOR, CARD_COLOR,
                                  ACCENT_COLOR, SECONDARY_COLOR, DANGER_COLOR);

    setStyleSheet(main_style);
}

void MainWindow::setupControlPanel()
{
    control_group_ = new QGroupBox("🎮 CONTROL INTERFACE");
    auto layout = new QVBoxLayout(control_group_);
    layout->setSpacing(15);

    // Angle control with enhanced styling
    auto angle_frame = new QFrame();
    angle_frame->setFrameStyle(QFrame::StyledPanel);
    auto angle_layout = new QVBoxLayout(angle_frame);

    auto angle_label = new QLabel("TARGET ANGLE (°)");
    angle_label->setAlignment(Qt::AlignCenter);
    angle_label->setStyleSheet("font-size: 14pt; font-weight: bold; color: " + ACCENT_COLOR + ";");

    auto angle_control_layout = new QHBoxLayout();
    angle_slider_ = new QSlider(Qt::Horizontal);
    angle_slider_->setRange(-90, 90);
    angle_slider_->setValue(0);
    angle_spinbox_ = new QDoubleSpinBox();
    angle_spinbox_->setRange(-90.0, 90.0);
    angle_spinbox_->setSuffix(" °");
    angle_spinbox_->setValue(0.0);
    angle_spinbox_->setMinimumWidth(120);

    angle_control_layout->addWidget(angle_slider_, 3);
    angle_control_layout->addWidget(angle_spinbox_, 1);

    angle_layout->addWidget(angle_label);
    angle_layout->addLayout(angle_control_layout);

    // Velocity control
    auto velocity_frame = new QFrame();
    velocity_frame->setFrameStyle(QFrame::StyledPanel);
    auto velocity_layout = new QVBoxLayout(velocity_frame);

    auto velocity_label = new QLabel("MOTOR VELOCITY (rad/s)");
    velocity_label->setAlignment(Qt::AlignCenter);
    velocity_label->setStyleSheet("font-size: 14pt; font-weight: bold; color: " + ACCENT_COLOR + ";");

    auto velocity_control_layout = new QHBoxLayout();
    velocity_slider_ = new QSlider(Qt::Horizontal);
    velocity_slider_->setRange(-785, 785);
    velocity_slider_->setValue(0);
    velocity_slider_->setTracking(false); // Publish only on release to avoid noise
    velocity_spinbox_ = new QDoubleSpinBox();
    velocity_spinbox_->setRange(-785.0, 785.0);
    velocity_spinbox_->setSuffix(" rad/s");
    velocity_spinbox_->setValue(0.0);
    velocity_spinbox_->setMinimumWidth(120);

    velocity_control_layout->addWidget(velocity_slider_, 3);
    velocity_control_layout->addWidget(velocity_spinbox_, 1);

    velocity_layout->addWidget(velocity_label);
    velocity_layout->addLayout(velocity_control_layout);

    // Control buttons with enhanced styling
    auto button_layout = new QHBoxLayout();
    stop_btn_ = new QPushButton("🛑 EMERGENCY STOP");
    stop_btn_->setObjectName("stopButton");
    stop_btn_->setMinimumHeight(50);

    stabilize_btn_ = new QPushButton("⚖️ STABILIZE");
    stabilize_btn_->setMinimumHeight(50);

    refresh_btn_ = new QPushButton("🔄 REFRESH");
    refresh_btn_->setMinimumHeight(50);

    button_layout->addWidget(stop_btn_);
    button_layout->addWidget(stabilize_btn_);
    button_layout->addWidget(refresh_btn_);

    // Add to main layout
    layout->addWidget(angle_frame);
    layout->addWidget(velocity_frame);
    layout->addLayout(button_layout);
    layout->addStretch();

    // Connect signals with thread-safe connections
    connect(angle_slider_, &QSlider::valueChanged, this, &MainWindow::onAngleSliderChanged, Qt::QueuedConnection);
    connect(velocity_slider_, &QSlider::valueChanged, this, &MainWindow::onVelocitySliderChanged, Qt::QueuedConnection);
    connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::onStopClicked, Qt::QueuedConnection);
    connect(stabilize_btn_, &QPushButton::clicked, this, &MainWindow::onStabilizeClicked, Qt::QueuedConnection);
    connect(refresh_btn_, &QPushButton::clicked, this, &MainWindow::onRefreshClicked, Qt::QueuedConnection);

    // FIXED: Correct Qt6 syntax for spinbox connections with lambdas
    // For Qt6, when using lambdas with connection type, we need to use the 5-parameter version
    connect(angle_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            { angle_slider_->setValue(static_cast<int>(value)); }, Qt::QueuedConnection);

    connect(velocity_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value)
            { velocity_slider_->setValue(static_cast<int>(value)); }, Qt::QueuedConnection);
}
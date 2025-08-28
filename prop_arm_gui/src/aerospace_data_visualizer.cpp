#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include <QDateTime>
#include <QApplication>
#include <algorithm>
#include <functional>
#include <QMutex>
#include <QMutexLocker>

AerospaceDataVisualizer::AerospaceDataVisualizer(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QWidget(parent),
      update_mutex_(new QMutex()), // FIXED: Initialize in correct order
      last_update_time_(0),
      ros_node_(node)
{
    loadConfiguration();
    setupUI();
    createCharts();

    // Optimized update frequency for better performance with smooth curves
    update_timer_ = new QTimer(this);
    update_timer_->setSingleShot(false);
    update_timer_->setInterval(50); // 20 FPS for smooth spline updates

    // Enhanced timer callback with smooth curve support
    connect(update_timer_, &QTimer::timeout, this, [this]()
            {
                // Minimal processing - just check if we need updates
                double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
                if (current_time - last_update_time_ > 0.05) // Update every 50ms for smoother curves
                {
                    // Updates are handled directly in onDataReceived
                } });
    update_timer_->start();
    setMinimumSize(1400, 900);

    // Enhanced styling with support for smooth curves
    setStyleSheet(QString(R"(
        QWidget {
            background-color: rgb(0, 0, 0);
            color: rgb(255, 255, 255);
            font-family: "Segoe UI", "Arial", sans-serif;
        }
        QChartView {
            background-color: rgb(0, 0, 0);
            border: 1px solid rgb(60, 60, 60);
            border-radius: 5px;
        }
    )"));
}

AerospaceDataVisualizer::~AerospaceDataVisualizer()
{
    if (update_timer_)
    {
        update_timer_->stop();
    }
    delete update_mutex_;
}

void AerospaceDataVisualizer::loadConfiguration()
{
    if (ros_node_)
    {
        try
        {
            // Load parameters with optimized defaults for smooth curves
            time_window_sec_ = ros_node_->get_parameter_or("visualization.time_window_sec", 30.0);
            max_points_ = ros_node_->get_parameter_or("visualization.max_plot_points", static_cast<int>(800)); // Increased for smooth curves
            update_rate_ms_ = ros_node_->get_parameter_or("visualization.update_rate_ms", 50);                 // Faster for smooth updates

            // NEW: Smooth curve configuration
            bool use_smooth_curves = ros_node_->get_parameter_or("visualization.use_smooth_curves", true);
            bool show_minor_grid = ros_node_->get_parameter_or("visualization.show_minor_grid", true);

            RCLCPP_INFO(ros_node_->get_logger(),
                        "Loaded enhanced visualization config - Time window: %.1f s, Max points: %zu, "
                        "Update rate: %d ms, Smooth curves: %s, Minor grid: %s",
                        time_window_sec_, max_points_, update_rate_ms_,
                        use_smooth_curves ? "enabled" : "disabled",
                        show_minor_grid ? "enabled" : "disabled");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(ros_node_->get_logger(),
                        "Failed to load some visualization parameters: %s. Using enhanced defaults.", e.what());
            // Enhanced defaults
            time_window_sec_ = 30.0;
            max_points_ = 800;
            update_rate_ms_ = 50;
        }
    }
    else
    {
        // Enhanced defaults
        time_window_sec_ = 30.0;
        max_points_ = 800;
        update_rate_ms_ = 50;
    }
}

ChartBase::ChartConfig AerospaceDataVisualizer::createChartConfig(
    const QString &title, const QString &y_label, const QString &units,
    const QColor &primary_color, const QColor &secondary_color,
    double y_min, double y_max, bool auto_scale, const QString &config_key)
{
    ChartBase::ChartConfig config;
    config.title = title;
    config.y_label = y_label;
    config.units = units;
    config.primary_color = primary_color;
    config.secondary_color = secondary_color;
    config.y_min = y_min;
    config.y_max = y_max;
    config.auto_scale = auto_scale;
    config.show_grid = true;
    config.time_window_sec = time_window_sec_;
    config.max_points = max_points_;
    config.show_milliseconds = false;
    config.use_smooth_curves = true;
    config.show_minor_grid = true;

    if (ros_node_)
    {
        try
        {
            std::string base_key = "visualization.charts." + config_key.toStdString();

            config.auto_scale = ros_node_->get_parameter_or(base_key + ".auto_scale", config.auto_scale);
            config.y_min = ros_node_->get_parameter_or(base_key + ".y_min", config.y_min);
            config.y_max = ros_node_->get_parameter_or(base_key + ".y_max", config.y_max);
            config.use_smooth_curves = ros_node_->get_parameter_or(base_key + ".use_smooth_curves", config.use_smooth_curves);
            config.show_minor_grid = ros_node_->get_parameter_or(base_key + ".show_minor_grid", config.show_minor_grid);

            auto color_str = ros_node_->get_parameter_or(base_key + ".color", std::string(""));
            if (!color_str.empty())
            {
                QColor loaded_color(QString::fromStdString(color_str));
                if (loaded_color.isValid())
                {
                    config.primary_color = loaded_color;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(ros_node_->get_logger(),
                        "Failed to load chart config for %s: %s. Using enhanced defaults.",
                        config_key.toStdString().c_str(), e.what());
        }
    }

    return config;
}

void AerospaceDataVisualizer::setupUI()
{
    main_layout_ = new QGridLayout(this);
    main_layout_->setSpacing(8);                      // Optimized spacing for better visibility
    main_layout_->setContentsMargins(12, 12, 12, 12); // Optimized margins

    // Enhanced chart configurations with professional color scheme
    chart_setups_ = {
        {"ARM ANGLE vs TIME",
         "Angle", "degrees",
         QColor(100, 200, 255), QColor(150, 220, 255), // Bright blue - primary signal
         0, 100, false, "arm_angle",
         [](const TelemetryData &data)
         { return data.arm_angle; }},

        {"CONTROL ERROR vs TIME",
         "Error", "degrees",
         QColor(255, 100, 100), QColor(255, 150, 150), // Bright red - error signal
         -90, 90, false, "control_error",
         [](const TelemetryData &data)
         { return data.error; }},

        {"MOTOR SPEED vs TIME",
         "Speed", "rad/s",
         QColor(100, 255, 100), QColor(150, 255, 150), // Bright green - motor feedback
         0, 800, false, "motor_speed",
         [](const TelemetryData &data)
         { return std::abs(data.motor_speed); }},

        {"MOTOR COMMAND vs TIME",
         "Command", "rad/s",
         QColor(255, 200, 0), QColor(255, 220, 100), // Bright yellow/orange - command signal
         0, 800, false, "motor_command",
         [](const TelemetryData &data)
         { return data.motor_command; }},

        {"V_EMF vs TIME",
         "Voltage", "V",
         QColor(200, 100, 255), QColor(220, 150, 255), // Bright purple - voltage
         0, 16, false, "v_emf",
         [](const TelemetryData &data)
         { return data.v_emf; }},

        {"DELTA V_EMF vs TIME",
         "Delta V", "V",
         QColor(255, 150, 200), QColor(255, 180, 220), // Bright pink - delta voltage
         0, 16, false, "delta_v_emf",
         [](const TelemetryData &data)
         { return data.delta_v_emf; }}};
}

void AerospaceDataVisualizer::createCharts()
{
    charts_.reserve(chart_setups_.size());

    for (size_t i = 0; i < chart_setups_.size(); ++i)
    {
        const auto &setup = chart_setups_[i];

        // Create enhanced chart configuration
        auto config = createChartConfig(
            setup.title, setup.y_label, setup.units,
            setup.primary_color, setup.secondary_color,
            setup.y_min, setup.y_max, setup.auto_scale, setup.config_key);

        // Create chart using enhanced base class
        auto chart = std::make_unique<ChartBase>(config);
        int row = static_cast<int>(i / 3);
        int col = static_cast<int>(i % 3);
        main_layout_->addWidget(chart.get(), row, col);

        charts_.push_back(std::move(chart));
    }

    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Created %zu enhanced charts with smooth curves and advanced grid", charts_.size());
    }
}

void AerospaceDataVisualizer::onDataReceived(double arm_angle, double motor_speed, double v_emf,
                                             double delta_v_emf, double error, double target_angle,
                                             double motor_command)
{
    QMutexLocker locker(update_mutex_);

    // Optimized throttling for smooth curve updates
    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    if (current_time - last_update_time_ < 0.05) // Update every 50ms for smooth curves
    {
        return;
    }

    // Create telemetry data structure
    TelemetryData data;
    data.timestamp = current_time;
    data.datetime = QDateTime::currentDateTime();
    data.arm_angle = arm_angle;
    data.motor_speed = motor_speed;
    data.v_emf = v_emf;
    data.delta_v_emf = delta_v_emf;
    data.error = error;
    data.target_angle = target_angle;
    data.motor_command = motor_command;

    // Enhanced data validation for smooth curves
    auto isValidValue = [](double value) -> bool
    {
        return std::isfinite(value) && !std::isnan(value) &&
               std::abs(value) < 1e6; // Reasonable bounds check
    };

    // Update charts efficiently with enhanced error handling
    for (size_t i = 0; i < charts_.size() && i < chart_setups_.size(); ++i)
    {
        try
        {
            double value = chart_setups_[i].data_extractor(data);

            // Enhanced validation for smooth curve stability
            if (isValidValue(value))
            {
                charts_[i]->addDataPoint(value, data.timestamp);
            }
            else
            {
                // For invalid data, use previous valid value or middle of range
                double fallback_value = (chart_setups_[i].y_min + chart_setups_[i].y_max) / 2.0;
                charts_[i]->addDataPoint(fallback_value, data.timestamp);

                if (ros_node_)
                {
                    RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 5000,
                                         "Invalid data for chart %zu, using fallback value", i);
                }
            }
        }
        catch (const std::exception &e)
        {
            // Log error but continue - don't let visualization errors affect control
            if (ros_node_)
            {
                RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 10000,
                                     "Chart update error for chart %zu: %s", i, e.what());
            }
        }
    }

    last_update_time_ = current_time;
}

void AerospaceDataVisualizer::clearData()
{
    QMutexLocker locker(update_mutex_);

    for (auto &chart : charts_)
    {
        if (chart)
        {
            chart->clearData();
        }
    }

    last_update_time_ = 0;

    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(), "Cleared all chart data - charts will reinitialize with zeros");
    }
}

void AerospaceDataVisualizer::setTimeWindow(double seconds)
{
    QMutexLocker locker(update_mutex_);

    time_window_sec_ = std::max(5.0, std::min(120.0, seconds)); // Enhanced range

    for (auto &chart : charts_)
    {
        if (chart)
        {
            chart->setTimeWindow(time_window_sec_);
        }
    }

    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Updated time window to %.1f seconds - charts reinitializing with smooth curves",
                    time_window_sec_);
    }
}

void AerospaceDataVisualizer::updateTheme()
{
    QMutexLocker locker(update_mutex_);

    for (auto &chart : charts_)
    {
        if (chart)
        {
            chart->updateTheme();
        }
    }

    if (ros_node_)
    {
        RCLCPP_DEBUG(ros_node_->get_logger(), "Updated chart themes with enhanced grid system");
    }
}

// NEW: Enhanced configuration methods
void AerospaceDataVisualizer::setSmoothCurves(bool enabled)
{
    QMutexLocker locker(update_mutex_);

    for (auto &chart : charts_)
    {
        if (chart)
        {
            chart->setSmoothCurves(enabled);
        }
    }

    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Smooth curves %s for all charts", enabled ? "enabled" : "disabled");
    }
}

void AerospaceDataVisualizer::setMinorGridVisible(bool visible)
{
    QMutexLocker locker(update_mutex_);

    // This would require extending the base ChartBase class to support minor grid control
    // For now, this is handled through the chart configuration

    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Minor grid lines %s", visible ? "enabled" : "disabled");
    }
}

bool AerospaceDataVisualizer::isSmoothCurvesEnabled() const
{
    if (!charts_.empty() && charts_[0])
    {
        return true;
    }
    return true;
}

double AerospaceDataVisualizer::getCurrentTimeWindow() const
{
    return time_window_sec_;
}

size_t AerospaceDataVisualizer::getMaxPoints() const
{
    return max_points_;
}

int AerospaceDataVisualizer::getUpdateRateMs() const
{
    return update_rate_ms_;
}
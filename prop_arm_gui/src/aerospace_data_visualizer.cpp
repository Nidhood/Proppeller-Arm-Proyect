#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include <QDateTime>
#include <QApplication>
#include <algorithm>
#include <functional>
#include <QMutex>
#include <QMutexLocker>

AerospaceDataVisualizer::AerospaceDataVisualizer(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QWidget(parent),
      update_mutex_(new QMutex()),
      last_update_time_(0),
      ros_node_(node)
{
    loadConfiguration();
    setupUI();
    createCharts();

    update_timer_ = new QTimer(this);
    update_timer_->setSingleShot(false);
    update_timer_->setInterval(50);

    connect(update_timer_, &QTimer::timeout, this, [this]()
            {
                double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
                if (current_time - last_update_time_ > 0.05)
                {
                    // Update logic si es necesario
                } });
    update_timer_->start();

    setMinimumSize(1400, 600);

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
            time_window_sec_ = ros_node_->get_parameter_or("visualization.time_window_sec", 30.0);
            max_points_ = ros_node_->get_parameter_or("visualization.max_plot_points", static_cast<int>(800));
            update_rate_ms_ = ros_node_->get_parameter_or("visualization.update_rate_ms", 50);

            bool use_smooth_curves = ros_node_->get_parameter_or("visualization.use_smooth_curves", true);
            bool show_minor_grid = ros_node_->get_parameter_or("visualization.show_minor_grid", true);

            RCLCPP_INFO(ros_node_->get_logger(),
                        "Loaded visualization config - Time window: %.1f s, Max points: %zu, "
                        "Update rate: %d ms, Smooth curves: %s, Minor grid: %s",
                        time_window_sec_, max_points_, update_rate_ms_,
                        use_smooth_curves ? "enabled" : "disabled",
                        show_minor_grid ? "enabled" : "disabled");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(ros_node_->get_logger(),
                        "Failed to load some visualization parameters: %s. Using defaults.", e.what());
            time_window_sec_ = 30.0;
            max_points_ = 800;
            update_rate_ms_ = 50;
        }
    }
    else
    {
        time_window_sec_ = 30.0;
        max_points_ = 800;
        update_rate_ms_ = 50;
    }
}

ChartBase::ChartConfig AerospaceDataVisualizer::createChartConfig(
    const QString &title, const QString &y_label, const QString &units,
    const QColor &primary_color, const QColor &secondary_color, const QColor &sim_color,
    double y_min, double y_max, bool auto_scale, const QString &config_key)
{
    ChartBase::ChartConfig config;
    config.title = title;
    config.y_label = y_label;
    config.units = units;
    config.primary_color = primary_color;
    config.secondary_color = secondary_color;
    config.sim_color = sim_color;
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

            // NOTA: Para ángulo y velocidad, ignoramos auto_scale de parámetros
            // y forzamos rangos fijos
            if (config_key == "arm_angle" || config_key == "motor_velocity")
            {
                config.auto_scale = false;
                RCLCPP_INFO(ros_node_->get_logger(),
                            "Chart '%s': Fixed range [%.1f, %.1f] %s",
                            config_key.toStdString().c_str(),
                            config.y_min, config.y_max,
                            units.toStdString().c_str());
            }
            else
            {
                config.auto_scale = ros_node_->get_parameter_or(base_key + ".auto_scale", config.auto_scale);
            }

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
                        "Failed to load chart config for %s: %s. Using defaults.",
                        config_key.toStdString().c_str(), e.what());
        }
    }

    return config;
}

void AerospaceDataVisualizer::setupUI()
{
    main_layout_ = new QGridLayout(this);
    main_layout_->setSpacing(8);
    main_layout_->setContentsMargins(12, 12, 12, 12);

    // 4 GRÁFICAS: ARM ANGLE (0-180°), MOTOR VELOCITY (-800 a 800 rad/s), PWM INPUT (us), DUTY CYCLE (%)
    // Con extractores tanto para datos reales como simulados
    chart_setups_ = {
        {"ARM ANGLE vs TIME",
         "Angle", "degrees",
         QColor(100, 200, 255), QColor(150, 220, 255), QColor(255, 150, 100), // sim_color naranja
         0.0, 100.0, false,
         "arm_angle",
         [](const TelemetryData &data)
         { return data.arm_angle; },
         [](const TelemetryData &data)
         { return data.sim_arm_angle; }},

        {"MOTOR VELOCITY vs TIME",
         "Velocity", "rad/s",
         QColor(100, 255, 100), QColor(150, 255, 150), QColor(255, 200, 100), // sim_color amarillo
         0.0, 1600.0, false,
         "motor_velocity",
         [](const TelemetryData &data)
         { return data.motor_speed; },
         [](const TelemetryData &data)
         { return data.sim_motor_speed; }},

        {"PWM INPUT vs TIME",
         "PWM", "µs",
         QColor(255, 120, 200), QColor(255, 170, 220), QColor(200, 100, 255), // sim_color púrpura
         1000, 2000, false,
         "pwm_input",
         [](const TelemetryData &data)
         { return data.pwm_input_us; },
         [](const TelemetryData &data)
         { return data.sim_pwm_input_us; }},

        {"DUTY CYCLE vs TIME",
         "Duty", "%",
         QColor(255, 255, 0), QColor(255, 255, 100), QColor(100, 255, 255), // sim_color cyan
         5, 10, false,
         "duty_cycle",
         [](const TelemetryData &data)
         { return data.duty_cycle_percent; },
         [](const TelemetryData &data)
         { return data.sim_duty_cycle_percent; }}};
}

void AerospaceDataVisualizer::createCharts()
{
    charts_.reserve(chart_setups_.size());

    for (size_t i = 0; i < chart_setups_.size(); ++i)
    {
        const auto &setup = chart_setups_[i];

        auto config = createChartConfig(
            setup.title, setup.y_label, setup.units,
            setup.primary_color, setup.secondary_color, setup.sim_color,
            setup.y_min, setup.y_max, setup.auto_scale, setup.config_key);

        auto chart = std::make_unique<ChartBase>(config);

        // Layout para 4 gráficas en 2x2:
        // Row 0: ARM ANGLE (0,0), MOTOR VELOCITY (0,1)
        // Row 1: PWM INPUT (1,0), DUTY CYCLE (1,1)
        int row = static_cast<int>(i / 2);
        int col = static_cast<int>(i % 2);

        main_layout_->addWidget(chart.get(), row, col);
        charts_.push_back(std::move(chart));
    }

    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Created %zu charts in 2x2 layout with real + simulation overlay:",
                    charts_.size());
        RCLCPP_INFO(ros_node_->get_logger(), "  - Angle: FIXED [0, 180] degrees");
        RCLCPP_INFO(ros_node_->get_logger(), "  - Velocity: FIXED [-800, 800] rad/s");
        RCLCPP_INFO(ros_node_->get_logger(), "  - PWM Input: AUTO-SCALE");
        RCLCPP_INFO(ros_node_->get_logger(), "  - Duty Cycle: AUTO-SCALE");
    }
}

double AerospaceDataVisualizer::calculateDutyCycle(double pwm_us) const
{
    return (pwm_us / PWM_PERIOD_US) * 100.0;
}

void AerospaceDataVisualizer::onDataReceived(double arm_angle, double motor_speed, double pwm_input_us,
                                             double sim_arm_angle, double sim_motor_speed, double sim_pwm_input_us)
{
    QMutexLocker locker(update_mutex_);

    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    if (current_time - last_update_time_ < 0.05)
    {
        return;
    }

    // Calcular duty cycles
    double duty_cycle = calculateDutyCycle(pwm_input_us);
    double sim_duty_cycle = calculateDutyCycle(sim_pwm_input_us);

    // Create telemetry data structure
    TelemetryData data;
    data.timestamp = current_time;
    data.datetime = QDateTime::currentDateTime();

    // Datos reales
    data.arm_angle = arm_angle;
    data.motor_speed = motor_speed;
    data.pwm_input_us = pwm_input_us;
    data.duty_cycle_percent = duty_cycle;

    // Datos de simulación
    data.sim_arm_angle = sim_arm_angle;
    data.sim_motor_speed = sim_motor_speed;
    data.sim_pwm_input_us = sim_pwm_input_us;
    data.sim_duty_cycle_percent = sim_duty_cycle;

    // Validación de datos
    auto isValidValue = [](double value) -> bool
    {
        return std::isfinite(value) && !std::isnan(value) && std::abs(value) < 1e6;
    };

    // Update charts con AMBOS conjuntos de datos
    for (size_t i = 0; i < charts_.size() && i < chart_setups_.size(); ++i)
    {
        try
        {
            // Agregar datos REALES
            double real_value = chart_setups_[i].data_extractor(data);
            if (isValidValue(real_value))
            {
                charts_[i]->addDataPoint(real_value, data.timestamp);
            }
            else
            {
                double fallback_value = (chart_setups_[i].y_min + chart_setups_[i].y_max) / 2.0;
                charts_[i]->addDataPoint(fallback_value, data.timestamp);
            }

            // Agregar datos de SIMULACIÓN
            double sim_value = chart_setups_[i].sim_data_extractor(data);
            if (isValidValue(sim_value))
            {
                charts_[i]->addSimDataPoint(sim_value, data.timestamp);
            }
            else
            {
                double fallback_value = (chart_setups_[i].y_min + chart_setups_[i].y_max) / 2.0;
                charts_[i]->addSimDataPoint(fallback_value, data.timestamp);
            }
        }
        catch (const std::exception &e)
        {
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
        RCLCPP_INFO(ros_node_->get_logger(), "Cleared all chart data (real + simulation)");
    }
}

void AerospaceDataVisualizer::setTimeWindow(double seconds)
{
    QMutexLocker locker(update_mutex_);

    time_window_sec_ = std::max(5.0, std::min(120.0, seconds));

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
                    "Updated time window to %.1f seconds", time_window_sec_);
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
        RCLCPP_DEBUG(ros_node_->get_logger(), "Updated chart themes");
    }
}

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

    if (ros_node_)
    {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Minor grid lines %s", visible ? "enabled" : "disabled");
    }
}

bool AerospaceDataVisualizer::isSmoothCurvesEnabled() const
{
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
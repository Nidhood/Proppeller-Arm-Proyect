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
                }
            });
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
        }
        catch (const std::exception &e)
        {
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

            if (config_key == "arm_angle" || config_key == "motor_velocity" || 
                config_key == "pwm_input" || config_key == "duty_cycle")
            {
                config.auto_scale = false;
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
        }
    }

    return config;
}

void AerospaceDataVisualizer::setupUI()
{
    main_layout_ = new QGridLayout(this);
    main_layout_->setSpacing(8);
    main_layout_->setContentsMargins(12, 12, 12, 12);

    chart_setups_ = {
        {"ARM ANGLE vs TIME",
         "Angle", "degrees",
         QColor(100, 200, 255), QColor(150, 220, 255), QColor(255, 150, 100),
         0.0, 100.0, false,
         "arm_angle",
         [](const TelemetryData &data) { return data.arm_angle; },
         [](const TelemetryData &data) { return data.sim_arm_angle; }},

        {"MOTOR VELOCITY vs TIME",
         "Velocity", "rad/s",
         QColor(100, 255, 100), QColor(150, 255, 150), QColor(255, 200, 100),
         0.0, 1600.0, false,
         "motor_velocity",
         [](const TelemetryData &data) { return data.motor_speed; },
         [](const TelemetryData &data) { return data.sim_motor_speed; }},

        {"PWM INPUT vs TIME",
         "PWM", "µs",
         QColor(255, 120, 200), QColor(255, 170, 220), QColor(200, 100, 255),
         1000.0, 2000.0, false,
         "pwm_input",
         [](const TelemetryData &data) { return data.pwm_input_us; },
         [](const TelemetryData &data) { return data.sim_pwm_input_us; }},

        {"DUTY CYCLE vs TIME",
         "Duty", "%",
         QColor(255, 255, 0), QColor(255, 255, 100), QColor(100, 255, 255),
         5.0, 10.0, false,
         "duty_cycle",
         [](const TelemetryData &data) { return data.duty_cycle_percent; },
         [](const TelemetryData &data) { return data.sim_duty_cycle_percent; }}};
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

        int row = static_cast<int>(i / 2);
        int col = static_cast<int>(i % 2);

        main_layout_->addWidget(chart.get(), row, col);
        charts_.push_back(std::move(chart));
    }
}

double AerospaceDataVisualizer::calculateDutyCycle(double pwm_us) const
{
    return (pwm_us / PWM_PERIOD_US) * 100.0;
}

void AerospaceDataVisualizer::onDataReceived(double arm_angle,
                                             double motor_speed,
                                             double pwm_input_us,
                                             double sim_arm_angle,
                                             double sim_motor_speed,
                                             double sim_pwm_input_us)
{
    QMutexLocker locker(update_mutex_);

    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;

    double duty_cycle     = calculateDutyCycle(pwm_input_us);
    double sim_duty_cycle = calculateDutyCycle(sim_pwm_input_us);

    TelemetryData data;
    data.timestamp              = current_time;
    data.datetime               = QDateTime::currentDateTime();
    data.arm_angle              = arm_angle;
    data.motor_speed            = motor_speed;
    data.pwm_input_us           = pwm_input_us;
    data.duty_cycle_percent     = duty_cycle;
    data.sim_arm_angle          = sim_arm_angle;
    data.sim_motor_speed        = sim_motor_speed;
    data.sim_pwm_input_us       = sim_pwm_input_us;
    data.sim_duty_cycle_percent = sim_duty_cycle;

    auto isValidValue = [](double value) -> bool {
        return std::isfinite(value) && !std::isnan(value) &&
               std::abs(value) < 1e6;
    };

    for (size_t i = 0; i < charts_.size() && i < chart_setups_.size(); ++i)
    {
        try
        {
            double real_value = chart_setups_[i].data_extractor(data);
            if (isValidValue(real_value) && charts_[i])
            {
                charts_[i]->addDataPoint(real_value, data.timestamp);
            }

            double sim_value = chart_setups_[i].sim_data_extractor(data);
            if (isValidValue(sim_value) && charts_[i])
            {
                charts_[i]->addSimDataPoint(sim_value, data.timestamp);
            }
        }
        catch (const std::exception &)
        {
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
}

void AerospaceDataVisualizer::setMinorGridVisible(bool visible)
{
    (void)visible;
    QMutexLocker locker(update_mutex_);
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
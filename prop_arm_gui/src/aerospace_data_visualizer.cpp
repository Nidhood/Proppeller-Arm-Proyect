#include "prop_arm_gui/aerospace_data_visualizer.hpp"
#include <QDateTime>
#include <QApplication>
#include <QFontDatabase>
#include <algorithm>
#include <cmath>

AerospaceDataVisualizer::AerospaceDataVisualizer(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QWidget(parent), ros_node_(node)
{
    start_time_ = QDateTime::currentMSecsSinceEpoch() / 1000.0;

    // Load configuration from ROS parameters
    loadConfiguration();

    setupUI();
    createCharts();
    applyAerospaceTheme();

    // Update timer for smooth animations - now configurable
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &AerospaceDataVisualizer::updateCharts);
    update_timer_->start(update_rate_ms_);

    setMinimumSize(1400, 900);
    setStyleSheet(QString(R"(
        QWidget {
            background-color: rgb(%1, %2, %3);
            color: rgb(%4, %5, %6);
        }
        QChartView {
            background-color: rgb(%1, %2, %3);
            border: 2px solid rgb(%7, %8, %9);
            border-radius: 8px;
        }
    )")
                      .arg(BACKGROUND_DARK.red())
                      .arg(BACKGROUND_DARK.green())
                      .arg(BACKGROUND_DARK.blue())
                      .arg(TEXT_COLOR.red())
                      .arg(TEXT_COLOR.green())
                      .arg(TEXT_COLOR.blue())
                      .arg(GRID_COLOR.red())
                      .arg(GRID_COLOR.green())
                      .arg(GRID_COLOR.blue()));
}

void AerospaceDataVisualizer::loadConfiguration()
{
    // Set default colors first
    BACKGROUND_DARK = QColor(8, 10, 15);
    GRID_COLOR = QColor(25, 35, 45, 120);
    TEXT_COLOR = QColor(220, 230, 240);
    NEON_BLUE = QColor(0, 150, 255);
    NEON_CYAN = QColor(0, 255, 200);
    NEON_GREEN = QColor(50, 255, 100);
    NEON_ORANGE = QColor(255, 150, 0);
    NEON_PURPLE = QColor(180, 100, 255);
    NEON_RED = QColor(255, 80, 80);
    ACCENT_GLOW = QColor(0, 200, 255, 100);

    if (ros_node_)
    {
        // Load visualization parameters
        time_window_sec_ = ros_node_->get_parameter_or("visualization.time_window_sec", 60.0);
        max_points_ = ros_node_->get_parameter_or("visualization.max_plot_points", static_cast<int>(2000));
        update_rate_ms_ = ros_node_->get_parameter_or("visualization.update_rate_ms", 50);

        // Load color scheme from parameters
        try
        {
            auto bg_color_str = ros_node_->get_parameter_or("colors.background_dark", std::string("#080a0f"));
            BACKGROUND_DARK = QColor(QString::fromStdString(bg_color_str));

            auto grid_color_str = ros_node_->get_parameter_or("colors.grid_color", std::string("#19232d"));
            GRID_COLOR = QColor(QString::fromStdString(grid_color_str));
            GRID_COLOR.setAlpha(120);

            auto text_color_str = ros_node_->get_parameter_or("colors.text_color", std::string("#dce6f0"));
            TEXT_COLOR = QColor(QString::fromStdString(text_color_str));

            auto neon_blue_str = ros_node_->get_parameter_or("colors.neon_blue", std::string("#0096ff"));
            NEON_BLUE = QColor(QString::fromStdString(neon_blue_str));

            auto neon_cyan_str = ros_node_->get_parameter_or("colors.neon_cyan", std::string("#00ffcc"));
            NEON_CYAN = QColor(QString::fromStdString(neon_cyan_str));

            auto neon_green_str = ros_node_->get_parameter_or("colors.neon_green", std::string("#32ff64"));
            NEON_GREEN = QColor(QString::fromStdString(neon_green_str));

            auto neon_orange_str = ros_node_->get_parameter_or("colors.neon_orange", std::string("#ff9600"));
            NEON_ORANGE = QColor(QString::fromStdString(neon_orange_str));

            auto neon_purple_str = ros_node_->get_parameter_or("colors.neon_purple", std::string("#b464ff"));
            NEON_PURPLE = QColor(QString::fromStdString(neon_purple_str));

            auto neon_red_str = ros_node_->get_parameter_or("colors.neon_red", std::string("#ff5050"));
            NEON_RED = QColor(QString::fromStdString(neon_red_str));
        }
        catch (const std::exception &e)
        {
            if (ros_node_)
            {
                RCLCPP_WARN(ros_node_->get_logger(), "Failed to load color configuration: %s", e.what());
            }
        }
    }
}

void AerospaceDataVisualizer::loadChartConfiguration()
{
    if (!ros_node_)
        return;

    try
    {
        // Load chart-specific configurations
        for (auto &config : chart_configs_)
        {
            std::string base_key = "visualization.charts." + config.config_key.toStdString();

            // Load auto-scale setting
            config.auto_scale = ros_node_->get_parameter_or(base_key + ".auto_scale", config.auto_scale);

            // Load Y-axis limits
            config.y_min = ros_node_->get_parameter_or(base_key + ".y_min", config.y_min);
            config.y_max = ros_node_->get_parameter_or(base_key + ".y_max", config.y_max);

            // Load custom color if specified
            auto color_str = ros_node_->get_parameter_or(base_key + ".color", std::string(""));
            if (!color_str.empty())
            {
                config.primary_color = QColor(QString::fromStdString(color_str));
            }
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(ros_node_->get_logger(), "Failed to load chart configuration: %s", e.what());
    }
}

void AerospaceDataVisualizer::setupUI()
{
    main_layout_ = new QGridLayout(this);
    main_layout_->setSpacing(12);
    main_layout_->setContentsMargins(15, 15, 15, 15);

    // Initialize chart configurations with configuration keys
    chart_configs_ = {
        {"🎯 ARM ANGLE vs TIME", "Angle", "degrees", NEON_CYAN, NEON_BLUE, -90, 90, false, 0, "arm_angle"},
        {"⚠️ CONTROL ERROR vs TIME", "Error", "degrees", NEON_RED, NEON_ORANGE, -10, 10, true, 1, "control_error"},
        {"⚡ MOTOR SPEED vs TIME", "Speed", "rad/s", NEON_GREEN, NEON_CYAN, 0, 800, false, 2, "motor_speed"},
        {"🚀 MOTOR COMMAND vs TIME", "Command", "rad/s", NEON_PURPLE, NEON_BLUE, -800, 800, false, 3, "motor_command"},
        {"🔋 V_EMF vs TIME", "Voltage", "V", NEON_ORANGE, NEON_RED, -20, 20, true, 4, "v_emf"},
        {"⚡ ΔV_EMF vs TIME", "Delta V", "V", NEON_BLUE, NEON_PURPLE, -5, 5, true, 5, "delta_v_emf"}};

    // Load chart-specific configuration
    loadChartConfiguration();
}

void AerospaceDataVisualizer::createCharts()
{
    charts_.resize(chart_configs_.size());
    chart_views_.resize(chart_configs_.size());
    main_series_.resize(chart_configs_.size());
    smooth_series_.resize(chart_configs_.size());
    x_axes_.resize(chart_configs_.size());
    y_axes_.resize(chart_configs_.size());

    for (size_t i = 0; i < chart_configs_.size(); ++i)
    {
        const auto &config = chart_configs_[i];

        // Create chart
        charts_[i] = new QChart();
        setupChart(charts_[i], config.title, config.primary_color);

        // Create main series (solid line with glow effect)
        main_series_[i] = new QLineSeries();
        main_series_[i]->setName(config.y_label);

        // Create smooth series (spline for smoother curves with transparency)
        smooth_series_[i] = new QSplineSeries();
        smooth_series_[i]->setName(config.y_label + " Trend");

        // Style the main series with glow effect
        QPen main_pen(config.primary_color);
        main_pen.setWidth(3);
        main_pen.setCosmetic(true);
        main_pen.setCapStyle(Qt::RoundCap);
        main_pen.setJoinStyle(Qt::RoundJoin);
        main_series_[i]->setPen(main_pen);

        // Style the smooth series with glow and transparency
        QPen smooth_pen(config.secondary_color);
        smooth_pen.setWidth(2);
        smooth_pen.setCosmetic(true);
        smooth_pen.setCapStyle(Qt::RoundCap);
        smooth_series_[i]->setPen(smooth_pen);
        smooth_series_[i]->setOpacity(0.6);

        charts_[i]->addSeries(main_series_[i]);
        charts_[i]->addSeries(smooth_series_[i]);

        // Create axes with aerospace styling and configuration-based ranges
        x_axes_[i] = new QValueAxis();
        x_axes_[i]->setTitleText("TIME (seconds)");
        x_axes_[i]->setRange(0, time_window_sec_);
        x_axes_[i]->setTickCount(8);
        x_axes_[i]->setLabelFormat("%.1f");
        x_axes_[i]->setLabelsColor(TEXT_COLOR);
        x_axes_[i]->setTitleBrush(QBrush(config.primary_color));
        x_axes_[i]->setGridLineColor(GRID_COLOR);
        x_axes_[i]->setLinePenColor(config.primary_color);

        // Enhanced axis font
        QFont axis_font("Consolas", 9, QFont::Bold);
        x_axes_[i]->setLabelsFont(axis_font);
        QFont title_font("Consolas", 10, QFont::Bold);
        x_axes_[i]->setTitleFont(title_font);

        y_axes_[i] = new QValueAxis();
        y_axes_[i]->setTitleText(config.y_label.toUpper() + " (" + config.units + ")");
        // Use configured Y-axis range
        y_axes_[i]->setRange(config.y_min, config.y_max);
        y_axes_[i]->setTickCount(10);
        y_axes_[i]->setLabelFormat("%.2f");
        y_axes_[i]->setLabelsColor(TEXT_COLOR);
        y_axes_[i]->setTitleBrush(QBrush(config.primary_color));
        y_axes_[i]->setGridLineColor(GRID_COLOR);
        y_axes_[i]->setLinePenColor(config.primary_color);
        y_axes_[i]->setLabelsFont(axis_font);
        y_axes_[i]->setTitleFont(title_font);

        charts_[i]->addAxis(x_axes_[i], Qt::AlignBottom);
        charts_[i]->addAxis(y_axes_[i], Qt::AlignLeft);
        main_series_[i]->attachAxis(x_axes_[i]);
        main_series_[i]->attachAxis(y_axes_[i]);
        smooth_series_[i]->attachAxis(x_axes_[i]);
        smooth_series_[i]->attachAxis(y_axes_[i]);

        // Create chart view with enhanced settings
        chart_views_[i] = new QChartView(charts_[i]);
        chart_views_[i]->setRenderHint(QPainter::Antialiasing);
        chart_views_[i]->setRenderHint(QPainter::SmoothPixmapTransform);
        chart_views_[i]->setMinimumSize(460, 300);
        chart_views_[i]->setMaximumHeight(350);

        // Add subtle drop shadow effect
        chart_views_[i]->setStyleSheet(QString(R"(
            QChartView {
                background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                    stop: 0 rgb(%1, %2, %3),
                    stop: 1 rgb(%4, %5, %6));
                border: 2px solid rgb(%7, %8, %9);
                border-radius: 12px;
                margin: 3px;
            }
        )")
                                           .arg(BACKGROUND_DARK.lighter(115).red())
                                           .arg(BACKGROUND_DARK.lighter(115).green())
                                           .arg(BACKGROUND_DARK.lighter(115).blue())
                                           .arg(BACKGROUND_DARK.darker(115).red())
                                           .arg(BACKGROUND_DARK.darker(115).green())
                                           .arg(BACKGROUND_DARK.darker(115).blue())
                                           .arg(config.primary_color.red())
                                           .arg(config.primary_color.green())
                                           .arg(config.primary_color.blue()));

        // Add to layout (3x2 grid for professional aerospace dashboard look)
        int row = i / 3;
        int col = i % 3;
        main_layout_->addWidget(chart_views_[i], row, col);
    }
}

void AerospaceDataVisualizer::setupChart(QChart *chart, const QString &title, const QColor &accentColor)
{
    chart->setTitle(title);
    chart->setAnimationOptions(QChart::NoAnimation); // Disabled for real-time performance

    // Background with sophisticated gradient
    QLinearGradient backgroundGradient;
    backgroundGradient.setStart(QPointF(0, 0));
    backgroundGradient.setFinalStop(QPointF(0, 1));
    backgroundGradient.setColorAt(0, BACKGROUND_DARK.lighter(105));
    backgroundGradient.setColorAt(0.5, BACKGROUND_DARK);
    backgroundGradient.setColorAt(1, BACKGROUND_DARK.darker(105));
    backgroundGradient.setCoordinateMode(QGradient::ObjectBoundingMode);
    chart->setBackgroundBrush(QBrush(backgroundGradient));

    // Plot area with subtle radial glow
    QRadialGradient plotGradient;
    plotGradient.setCenter(0.5, 0.5);
    plotGradient.setRadius(0.8);
    plotGradient.setColorAt(0, QColor(15, 20, 30, 200));
    plotGradient.setColorAt(0.7, QColor(8, 12, 18, 250));
    plotGradient.setColorAt(1, BACKGROUND_DARK);
    plotGradient.setCoordinateMode(QGradient::ObjectBoundingMode);
    chart->setPlotAreaBackgroundBrush(QBrush(plotGradient));
    chart->setPlotAreaBackgroundVisible(true);

    // Title styling with glow effect
    chart->setTitleBrush(QBrush(accentColor));
    QFont titleFont("Orbitron", 13, QFont::Bold); // Futuristic font
    // Fallback to system fonts if Orbitron not available
    if (!QFontDatabase::families().contains("Orbitron"))
    {
        titleFont = QFont("Arial", 13, QFont::Bold);
    }
    chart->setTitleFont(titleFont);

    // Legend with aerospace styling
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->legend()->setLabelColor(TEXT_COLOR);
    chart->legend()->setFont(QFont("Consolas", 9));
    chart->legend()->setBackgroundVisible(false);
    chart->legend()->setBorderColor(Qt::transparent);

    // Enhanced margins for professional look
    chart->setMargins(QMargins(15, 15, 15, 15));

    // Add subtle border around the chart
    chart->setBackgroundRoundness(8);
}

void AerospaceDataVisualizer::applyAerospaceTheme()
{
    // Additional theming could be applied here
    // This method allows for dynamic theme switching if needed
}

void AerospaceDataVisualizer::addDataPoint(const TelemetryPoint &point)
{
    data_points_.push_back(point);

    // Remove old data points outside time window
    double current_time = point.timestamp;
    while (!data_points_.empty() &&
           (current_time - data_points_.front().timestamp) > time_window_sec_)
    {
        data_points_.pop_front();
    }

    // Limit total points for performance (now configurable)
    while (data_points_.size() > max_points_)
    {
        data_points_.pop_front();
    }
}

void AerospaceDataVisualizer::updateCharts()
{
    if (data_points_.empty())
        return;

    // Prepare data vectors for each chart
    std::vector<std::vector<QPointF>> series_data(chart_configs_.size());

    // Extract data for each series
    for (const auto &point : data_points_)
    {
        double rel_time = point.timestamp - start_time_;

        series_data[0].emplace_back(rel_time, point.arm_angle);
        series_data[1].emplace_back(rel_time, point.error);
        series_data[2].emplace_back(rel_time, std::abs(point.motor_speed)); // Show absolute value for speed
        series_data[3].emplace_back(rel_time, point.motor_command);
        series_data[4].emplace_back(rel_time, point.v_emf);
        series_data[5].emplace_back(rel_time, point.delta_v_emf);
    }

    // Update each series
    for (size_t i = 0; i < chart_configs_.size() && i < series_data.size(); ++i)
    {
        if (!series_data[i].empty())
        {
            // Convert std::vector<QPointF> to QList<QPointF> for Qt6 compatibility
            QList<QPointF> qt_series_data;
            qt_series_data.reserve(series_data[i].size());
            for (const auto &point : series_data[i])
            {
                qt_series_data.append(point);
            }

            // Update main series
            main_series_[i]->replace(qt_series_data);

            // Create smoothed data for the smooth series
            if (series_data[i].size() > 2)
            {
                QList<QPointF> smoothed_data;
                for (size_t j = 1; j < series_data[i].size() - 1; ++j)
                {
                    double smooth_y = (series_data[i][j - 1].y() + series_data[i][j].y() + series_data[i][j + 1].y()) / 3.0;
                    smoothed_data.append(QPointF(series_data[i][j].x(), smooth_y));
                }
                if (!smoothed_data.empty())
                {
                    smooth_series_[i]->replace(smoothed_data);
                }
            }

            // Auto-scale Y axis if enabled in configuration
            if (chart_configs_[i].auto_scale && !series_data[i].empty())
            {
                auto minmax = std::minmax_element(series_data[i].begin(), series_data[i].end(),
                                                  [](const QPointF &a, const QPointF &b)
                                                  { return a.y() < b.y(); });

                if (minmax.first != series_data[i].end())
                {
                    double y_range = minmax.second->y() - minmax.first->y();
                    double margin = std::max(0.1, y_range * 0.15); // 15% margin

                    double new_min = minmax.first->y() - margin;
                    double new_max = minmax.second->y() + margin;

                    // Smooth axis transitions
                    double current_min = y_axes_[i]->min();
                    double current_max = y_axes_[i]->max();

                    double smooth_min = current_min + (new_min - current_min) * 0.1;
                    double smooth_max = current_max + (new_max - current_max) * 0.1;

                    y_axes_[i]->setRange(smooth_min, smooth_max);
                }
            }

            // FIXED: Update X axis to show current time window properly - full range utilization
            if (!data_points_.empty())
            {
                double current_time = data_points_.back().timestamp - start_time_;

                // Ensure we use the full chart width by properly setting the time window
                if (current_time <= time_window_sec_)
                {
                    // For early data, show from 0 to time_window_sec_
                    x_axes_[i]->setRange(0.0, time_window_sec_);
                }
                else
                {
                    // For later data, show a sliding window
                    double window_start = current_time - time_window_sec_;
                    double window_end = current_time;
                    x_axes_[i]->setRange(window_start, window_end);
                }
            }
        }
    }
}

void AerospaceDataVisualizer::onDataReceived(double arm_angle, double motor_speed, double v_emf,
                                             double delta_v_emf, double error, double target_angle,
                                             double motor_command)
{
    TelemetryPoint point;
    point.timestamp = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    point.datetime = QDateTime::currentDateTime();
    point.arm_angle = arm_angle;
    point.motor_speed = motor_speed;
    point.v_emf = v_emf;
    point.delta_v_emf = delta_v_emf;
    point.error = error;
    point.target_angle = target_angle;
    point.motor_command = motor_command;

    addDataPoint(point);
}

void AerospaceDataVisualizer::clearData()
{
    data_points_.clear();
    start_time_ = QDateTime::currentMSecsSinceEpoch() / 1000.0;

    // Clear all series
    for (auto *series : main_series_)
    {
        if (series)
            series->clear();
    }
    for (auto *series : smooth_series_)
    {
        if (series)
            series->clear();
    }

    // Reset X-axis ranges
    for (auto *axis : x_axes_)
    {
        if (axis)
        {
            axis->setRange(0, time_window_sec_);
        }
    }
}

void AerospaceDataVisualizer::setTimeWindow(double seconds)
{
    time_window_sec_ = std::max(5.0, std::min(300.0, seconds)); // Clamp between 5 and 300 seconds

    // Update all X axes
    for (auto *axis : x_axes_)
    {
        if (axis && !data_points_.empty())
        {
            double current_time = data_points_.back().timestamp - start_time_;
            if (current_time <= time_window_sec_)
            {
                axis->setRange(0.0, time_window_sec_);
            }
            else
            {
                double window_start = current_time - time_window_sec_;
                double window_end = current_time;
                axis->setRange(window_start, window_end);
            }
        }
        else if (axis)
        {
            axis->setRange(0, time_window_sec_);
        }
    }
}

void AerospaceDataVisualizer::updateTheme()
{
    // Reload configuration and apply
    loadConfiguration();
    loadChartConfiguration();
    applyAerospaceTheme();

    // Force repaint of all charts
    for (auto *view : chart_views_)
    {
        if (view)
            view->update();
    }
}
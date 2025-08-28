#include "prop_arm_gui/real_time_plot.hpp"
#include <QDateTime>
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include <algorithm>
#include <numeric>

// Static member definitions for aerospace theme colors
const QColor RealTimePlot::GRID_COLOR = QColor(64, 128, 255, 80); // Semi-transparent blue
const QColor RealTimePlot::BACKGROUND_COLOR = QColor(15, 23, 42); // Dark slate
const QColor RealTimePlot::TEXT_COLOR = QColor(241, 245, 249);    // Light text
const QColor RealTimePlot::AXIS_COLOR = QColor(71, 85, 105);      // Slate gray
const QColor RealTimePlot::PLOT_BACKGROUND = QColor(30, 41, 59);  // Darker slate

RealTimePlot::RealTimePlot(const QString &title, const QString &y_label,
                           const QString &units, QWidget *parent)
    : QWidget(parent),
      title_(title), y_label_(y_label), units_(units),
      max_points_(1000), time_window_sec_(30.0),
      line_color_(QColor(59, 130, 246)), line_width_(2),
      auto_scale_(true), y_min_(0.0), y_max_(100.0),
      show_statistics_(true),
      min_value_(0.0), max_value_(0.0), avg_value_(0.0), current_value_(0.0)
{
    setupUI();
    setupChart();
    setupControls();
    applyAerospaceTheme();

    start_time_ = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    start_timestamp_ = QDateTime::currentDateTime();
}

void RealTimePlot::setupUI()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setSpacing(5);
    main_layout_->setContentsMargins(5, 5, 5, 5);

    // Chart view
    chart_view_ = new QChartView();
    chart_view_->setRenderHint(QPainter::Antialiasing);
    chart_view_->setMinimumHeight(200);

    main_layout_->addWidget(chart_view_);
}

void RealTimePlot::setupChart()
{
    chart_ = new QChart();
    chart_->setTitle(title_);
    chart_->setAnimationOptions(QChart::NoAnimation);

    // Create series
    series_ = new QLineSeries();
    series_->setName(y_label_);
    chart_->addSeries(series_);

    // Create axes
    x_axis_ = new QValueAxis();
    x_axis_->setTitleText("Time (s)");
    x_axis_->setTickCount(6);
    x_axis_->setLabelFormat("%.1f");

    y_axis_ = new QValueAxis();
    y_axis_->setTitleText(y_label_ + (units_.isEmpty() ? "" : " (" + units_ + ")"));
    y_axis_->setTickCount(6);
    y_axis_->setLabelFormat("%.2f");

    chart_->addAxis(x_axis_, Qt::AlignBottom);
    chart_->addAxis(y_axis_, Qt::AlignLeft);
    series_->attachAxis(x_axis_);
    series_->attachAxis(y_axis_);

    chart_view_->setChart(chart_);
}

void RealTimePlot::setupControls()
{
    controls_group_ = new QGroupBox("Plot Controls");
    controls_layout_ = new QHBoxLayout(controls_group_);

    // Autoscale checkbox
    autoscale_cb_ = new QCheckBox("Auto Scale");
    autoscale_cb_->setChecked(auto_scale_);
    connect(autoscale_cb_, &QCheckBox::toggled, this, &RealTimePlot::onAutoscaleToggled);

    // Max points spinbox
    controls_layout_->addWidget(new QLabel("Max Points:"));
    max_points_sb_ = new QSpinBox();
    max_points_sb_->setRange(100, 10000);
    max_points_sb_->setValue(max_points_);
    max_points_sb_->setSingleStep(100);
    connect(max_points_sb_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &RealTimePlot::onMaxPointsChanged);

    // Time window spinbox
    controls_layout_->addWidget(new QLabel("Time Window (s):"));
    time_window_sb_ = new QSpinBox();
    time_window_sb_->setRange(5, 300);
    time_window_sb_->setValue(static_cast<int>(time_window_sec_));
    connect(time_window_sb_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &RealTimePlot::onTimeWindowChanged);

    // Control buttons
    clear_btn_ = new QPushButton("Clear");
    export_btn_ = new QPushButton("Export");
    connect(clear_btn_, &QPushButton::clicked, this, &RealTimePlot::onClearData);
    connect(export_btn_, &QPushButton::clicked, this, &RealTimePlot::onExportData);

    // Layout controls
    controls_layout_->addWidget(autoscale_cb_);
    controls_layout_->addWidget(max_points_sb_);
    controls_layout_->addWidget(time_window_sb_);
    controls_layout_->addStretch();
    controls_layout_->addWidget(clear_btn_);
    controls_layout_->addWidget(export_btn_);

    // Statistics group
    if (show_statistics_)
    {
        stats_group_ = new QGroupBox("Statistics");
        auto stats_layout = new QHBoxLayout(stats_group_);

        min_label_ = new QLabel("Min: --");
        max_label_ = new QLabel("Max: --");
        avg_label_ = new QLabel("Avg: --");
        current_label_ = new QLabel("Current: --");

        stats_layout->addWidget(min_label_);
        stats_layout->addWidget(max_label_);
        stats_layout->addWidget(avg_label_);
        stats_layout->addWidget(current_label_);
        stats_layout->addStretch();

        main_layout_->addWidget(stats_group_);
    }

    main_layout_->addWidget(controls_group_);
}

void RealTimePlot::applyAerospaceTheme()
{
    // Set aerospace colors
    chart_->setBackgroundBrush(QBrush(PLOT_BACKGROUND));
    chart_->setPlotAreaBackgroundBrush(QBrush(BACKGROUND_COLOR));
    chart_->setPlotAreaBackgroundVisible(true);

    // Grid
    x_axis_->setGridLineColor(GRID_COLOR);
    y_axis_->setGridLineColor(GRID_COLOR);
    x_axis_->setLabelsColor(TEXT_COLOR);
    y_axis_->setLabelsColor(TEXT_COLOR);
    x_axis_->setTitleBrush(QBrush(TEXT_COLOR));
    y_axis_->setTitleBrush(QBrush(TEXT_COLOR));

    // Series
    QPen pen(line_color_);
    pen.setWidth(line_width_);
    series_->setPen(pen);

    // Chart title
    chart_->setTitleBrush(QBrush(TEXT_COLOR));
    QFont title_font = chart_->titleFont();
    title_font.setPointSize(12);
    title_font.setBold(true);
    chart_->setTitleFont(title_font);
}

void RealTimePlot::addDataPoint(double value, double time_sec)
{
    if (time_sec < 0)
    {
        time_sec = QDateTime::currentMSecsSinceEpoch() / 1000.0 - start_time_;
    }

    QDateTime timestamp = start_timestamp_.addMSecs(static_cast<qint64>(time_sec * 1000));
    addDataPoint(value, timestamp);
}

void RealTimePlot::addDataPoint(double value, const QDateTime &timestamp)
{
    double time_sec = start_timestamp_.msecsTo(timestamp) / 1000.0;

    PlotPoint point;
    point.value = value;
    point.time_sec = time_sec;
    point.timestamp = timestamp;

    data_points_.push_back(point);
    current_value_ = value;

    // Remove old points based on time window
    double current_time = time_sec;
    while (!data_points_.empty() &&
           (current_time - data_points_.front().time_sec) > time_window_sec_)
    {
        data_points_.pop_front();
    }

    // Remove excess points
    while (data_points_.size() > static_cast<size_t>(max_points_))
    {
        data_points_.pop_front();
    }

    // Update series
    QVector<QPointF> points;
    points.reserve(static_cast<int>(data_points_.size()));

    for (const auto &p : data_points_)
    {
        points.append(QPointF(p.time_sec, p.value));
    }

    series_->replace(points);

    // Update axes
    updateAxes();
    updateStatistics();

    emit dataPointAdded(value);
}

void RealTimePlot::updateAxes()
{
    if (data_points_.empty())
        return;

    // X-axis (time)
    double max_time = data_points_.back().time_sec;
    double min_time = std::max(0.0, max_time - time_window_sec_);
    x_axis_->setRange(min_time, max_time);

    // Y-axis (value)
    if (auto_scale_)
    {
        auto minmax = std::minmax_element(data_points_.begin(), data_points_.end(),
                                          [](const PlotPoint &a, const PlotPoint &b)
                                          { return a.value < b.value; });

        if (minmax.first != data_points_.end() && minmax.second != data_points_.end())
        {
            double min_val = minmax.first->value;
            double max_val = minmax.second->value;

            // Add 10% margin
            double margin = std::max(0.1, (max_val - min_val) * 0.1);
            y_axis_->setRange(min_val - margin, max_val + margin);
        }
    }
    else
    {
        y_axis_->setRange(y_min_, y_max_);
    }
}

void RealTimePlot::updateStatistics()
{
    if (!show_statistics_ || data_points_.empty())
        return;

    // Calculate statistics
    auto minmax = std::minmax_element(data_points_.begin(), data_points_.end(),
                                      [](const PlotPoint &a, const PlotPoint &b)
                                      { return a.value < b.value; });

    min_value_ = minmax.first->value;
    max_value_ = minmax.second->value;

    double sum = std::accumulate(data_points_.begin(), data_points_.end(), 0.0,
                                 [](double acc, const PlotPoint &p)
                                 { return acc + p.value; });
    avg_value_ = sum / data_points_.size();

    // Update labels
    QString unit_str = units_.isEmpty() ? "" : " " + units_;
    min_label_->setText(QString("Min: %1%2").arg(min_value_, 0, 'f', 2).arg(unit_str));
    max_label_->setText(QString("Max: %1%2").arg(max_value_, 0, 'f', 2).arg(unit_str));
    avg_label_->setText(QString("Avg: %1%2").arg(avg_value_, 0, 'f', 2).arg(unit_str));
    current_label_->setText(QString("Current: %1%2").arg(current_value_, 0, 'f', 2).arg(unit_str));

    emit statisticsUpdated(min_value_, max_value_, avg_value_, current_value_);
}

void RealTimePlot::clearData()
{
    data_points_.clear();
    series_->clear();

    min_value_ = max_value_ = avg_value_ = current_value_ = 0.0;
    start_time_ = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    start_timestamp_ = QDateTime::currentDateTime();

    if (show_statistics_)
    {
        min_label_->setText("Min: --");
        max_label_->setText("Max: --");
        avg_label_->setText("Avg: --");
        current_label_->setText("Current: --");
    }
}

void RealTimePlot::setLineColor(const QColor &color)
{
    line_color_ = color;
    QPen pen(line_color_);
    pen.setWidth(line_width_);
    series_->setPen(pen);
}

void RealTimePlot::setLineWidth(int width)
{
    line_width_ = width;
    QPen pen(line_color_);
    pen.setWidth(line_width_);
    series_->setPen(pen);
}

void RealTimePlot::setYRange(double min, double max)
{
    y_min_ = min;
    y_max_ = max;
    if (!auto_scale_)
    {
        y_axis_->setRange(y_min_, y_max_);
    }
}

void RealTimePlot::setAutoScale(bool enabled)
{
    auto_scale_ = enabled;
    autoscale_cb_->setChecked(enabled);
    updateAxes();
}

void RealTimePlot::setMaxPoints(int max_points)
{
    max_points_ = max_points;
    max_points_sb_->setValue(max_points);

    // Remove excess points
    while (data_points_.size() > static_cast<size_t>(max_points_))
    {
        data_points_.pop_front();
    }
}

void RealTimePlot::setTimeWindow(double seconds)
{
    time_window_sec_ = seconds;
    time_window_sb_->setValue(static_cast<int>(seconds));
    updateAxes();
}

void RealTimePlot::setGridVisible(bool visible)
{
    x_axis_->setGridLineVisible(visible);
    y_axis_->setGridLineVisible(visible);
}

void RealTimePlot::setTheme(QChart::ChartTheme theme)
{
    chart_->setTheme(theme);
    applyAerospaceTheme(); // Reapply custom theme after setting Qt theme
}

void RealTimePlot::showStatistics(bool show)
{
    show_statistics_ = show;
    if (stats_group_)
    {
        stats_group_->setVisible(show);
    }
}

QPixmap RealTimePlot::getPlotImage()
{
    return chart_view_->grab();
}

void RealTimePlot::onAutoscaleToggled(bool enabled)
{
    auto_scale_ = enabled;
    updateAxes();
}

void RealTimePlot::onMaxPointsChanged(int value)
{
    setMaxPoints(value);
}

void RealTimePlot::onTimeWindowChanged(double seconds)
{
    setTimeWindow(seconds);
}

void RealTimePlot::onClearData()
{
    clearData();
}

void RealTimePlot::onExportData()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    "Export Plot Data",
                                                    QString("%1_data.csv").arg(title_.toLower().replace(" ", "_")),
                                                    "CSV Files (*.csv)");

    if (!filename.isEmpty())
    {
        exportToCsv(filename);
    }
}

void RealTimePlot::exportToCsv(const QString &filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this, "Export Error",
                             "Could not open file for writing: " + filename);
        return;
    }

    QTextStream out(&file);

    // Header
    out << "Timestamp,Time_Sec," << y_label_;
    if (!units_.isEmpty())
    {
        out << "_" << units_;
    }
    out << "\n";

    // Data
    for (const auto &point : data_points_)
    {
        out << point.timestamp.toString(Qt::ISODate) << ","
            << point.time_sec << ","
            << point.value << "\n";
    }

    file.close();

    QMessageBox::information(this, "Export Complete",
                             QString("Data exported to: %1\n%2 points saved")
                                 .arg(filename)
                                 .arg(data_points_.size()));
}
#pragma once

#include <QtWidgets>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QSpinBox>
#include <QGroupBox>
#include <QTimer>
#include <QDateTime>
#include <deque>
#include <memory>
#include <functional>

// Qt6 Charts includes
#include <QChart>
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>

struct PlotPoint
{
    double time_sec;
    double value;
    QDateTime timestamp;

    PlotPoint() : time_sec(0.0), value(0.0) {}
    PlotPoint(double t, double v, const QDateTime &ts)
        : time_sec(t), value(v), timestamp(ts) {}
};

class RealTimePlot : public QWidget
{
    Q_OBJECT

public:
    explicit RealTimePlot(const QString &title,
                          const QString &y_label = "Value",
                          const QString &units = "",
                          QWidget *parent = nullptr);
    ~RealTimePlot() = default;

    // Data management
    void addDataPoint(double value, double time_sec = -1);
    void addDataPoint(double value, const QDateTime &timestamp);
    void clearData();
    void setMaxPoints(int max_points);
    void setTimeWindow(double seconds);

    // Appearance
    void setLineColor(const QColor &color);
    void setLineWidth(int width);
    void setYRange(double min, double max);
    void setAutoScale(bool enabled);
    void setGridVisible(bool visible);
    void setTheme(QChart::ChartTheme theme);

    // Statistics
    void showStatistics(bool show);
    void updateStatistics();

    // Export
    void exportToCsv(const QString &filename);
    QPixmap getPlotImage();

signals:
    void dataPointAdded(double value);
    void statisticsUpdated(double min, double max, double avg, double current);

private slots:
    void onAutoscaleToggled(bool enabled);
    void onMaxPointsChanged(int value);
    void onTimeWindowChanged(double seconds);
    void onClearData();
    void onExportData();

private:
    void setupUI();
    void setupChart();
    void setupControls();
    void updateAxes();
    void applyAerospaceTheme();

    // Chart components - Qt6 style (no namespace prefix needed)
    QChart *chart_;
    QChartView *chart_view_;
    QLineSeries *series_;
    QValueAxis *x_axis_;
    QValueAxis *y_axis_;

    // Layout
    QVBoxLayout *main_layout_;
    QHBoxLayout *controls_layout_;
    QGroupBox *controls_group_;

    // Controls
    QCheckBox *autoscale_cb_;
    QSpinBox *max_points_sb_;
    QSpinBox *time_window_sb_;
    QPushButton *clear_btn_;
    QPushButton *export_btn_;

    // Statistics
    QLabel *min_label_;
    QLabel *max_label_;
    QLabel *avg_label_;
    QLabel *current_label_;
    QGroupBox *stats_group_;

    // Data storage
    std::deque<PlotPoint> data_points_;
    double start_time_;
    QDateTime start_timestamp_;

    // Configuration
    QString title_;
    QString y_label_;
    QString units_;
    int max_points_;
    double time_window_sec_;
    QColor line_color_;
    int line_width_;
    bool auto_scale_;
    double y_min_, y_max_;

    // Statistics
    bool show_statistics_;
    double min_value_, max_value_, avg_value_, current_value_;

    // Aerospace theme colors
    static const QColor GRID_COLOR;
    static const QColor BACKGROUND_COLOR;
    static const QColor TEXT_COLOR;
    static const QColor AXIS_COLOR;
    static const QColor PLOT_BACKGROUND;
};
#pragma once

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFrame>
#include <QTimer>
#include <QChart>
#include <QChartView>
#include <QLineSeries>
#include <QSplineSeries>
#include <QScatterSeries>
#include <QValueAxis>
#include <QDateTimeAxis>
#include <QPainter>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QFont>
#include <QDateTime>
#include <deque>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

struct TelemetryPoint
{
    double timestamp;
    double arm_angle;
    double motor_speed;
    double v_emf;
    double delta_v_emf;
    double error;
    double target_angle;
    double motor_command;
    QDateTime datetime;
};

struct ChartConfig
{
    QString title;
    QString y_label;
    QString units;
    QColor primary_color;
    QColor secondary_color;
    double y_min;
    double y_max;
    bool auto_scale;
    int chart_index;
    QString config_key;
};

class AerospaceDataVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit AerospaceDataVisualizer(std::shared_ptr<rclcpp::Node> node = nullptr, QWidget *parent = nullptr);
    ~AerospaceDataVisualizer() = default;

    void addDataPoint(const TelemetryPoint &point);
    void clearData();
    void setTimeWindow(double seconds);
    void updateTheme();
    void loadConfiguration();

public slots:
    void onDataReceived(double arm_angle, double motor_speed, double v_emf,
                        double delta_v_emf, double error, double target_angle,
                        double motor_command);

private slots:
    void updateCharts();

private:
    void setupUI();
    void createCharts();
    void applyAerospaceTheme();
    void setupChart(QChart *chart, const QString &title, const QColor &accentColor);
    void loadChartConfiguration();

    // Chart management
    std::vector<QChart *> charts_;
    std::vector<QChartView *> chart_views_;
    std::vector<QLineSeries *> main_series_;
    std::vector<QSplineSeries *> smooth_series_;
    std::vector<QValueAxis *> x_axes_;
    std::vector<QValueAxis *> y_axes_;

    // Data storage
    std::deque<TelemetryPoint> data_points_;
    double time_window_sec_ = 60.0;
    size_t max_points_ = 2000;
    double start_time_;
    int update_rate_ms_ = 50;

    // UI Components
    QGridLayout *main_layout_;
    QTimer *update_timer_;

    // Configuration
    std::vector<ChartConfig> chart_configs_;
    std::shared_ptr<rclcpp::Node> ros_node_;

    // Aerospace color scheme
    QColor BACKGROUND_DARK;
    QColor GRID_COLOR;
    QColor TEXT_COLOR;
    QColor NEON_BLUE;
    QColor NEON_CYAN;
    QColor NEON_GREEN;
    QColor NEON_ORANGE;
    QColor NEON_PURPLE;
    QColor NEON_RED;
    QColor ACCENT_GLOW;
};
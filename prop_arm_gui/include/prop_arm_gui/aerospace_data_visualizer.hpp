#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QTimer>
#include <QDateTime>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <functional>
#include <mutex>

#include "prop_arm_gui/chart_base.hpp"

struct TelemetryData
{
    double timestamp;
    QDateTime datetime;
    double arm_angle;
    double motor_speed;
    double pwm_input_us;
    double duty_cycle_percent;

    double sim_arm_angle;
    double sim_motor_speed;
    double sim_pwm_input_us;
    double sim_duty_cycle_percent;
};

class AerospaceDataVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit AerospaceDataVisualizer(std::shared_ptr<rclcpp::Node> node = nullptr, QWidget *parent = nullptr);
    ~AerospaceDataVisualizer();

    void onDataReceived(double arm_angle, double motor_speed, double pwm_input_us,
                        double sim_arm_angle, double sim_motor_speed, double sim_pwm_input_us);

    void clearData();
    void setTimeWindow(double seconds);
    void updateTheme();
    void setSmoothCurves(bool enabled);
    void setMinorGridVisible(bool visible);

    bool isSmoothCurvesEnabled() const;
    double getCurrentTimeWindow() const;
    size_t getMaxPoints() const;
    int getUpdateRateMs() const;

private:
    struct ChartSetup
    {
        QString title;
        QString y_label;
        QString units;
        QColor primary_color;
        QColor secondary_color;
        QColor sim_color;
        double y_min;
        double y_max;
        bool auto_scale;
        QString config_key;
        std::function<double(const TelemetryData &)> data_extractor;
        std::function<double(const TelemetryData &)> sim_data_extractor;
    };

    void loadConfiguration();
    void setupUI();
    void createCharts();

    double calculateDutyCycle(double pwm_us) const;

    ChartBase::ChartConfig createChartConfig(
        const QString &title, const QString &y_label, const QString &units,
        const QColor &primary_color, const QColor &secondary_color, const QColor &sim_color,
        double y_min, double y_max, bool auto_scale, const QString &config_key);

    QGridLayout *main_layout_;
    std::vector<std::unique_ptr<ChartBase>> charts_;
    std::vector<ChartSetup> chart_setups_;

    QTimer *update_timer_;
    QMutex *update_mutex_;
    double last_update_time_;

    std::shared_ptr<rclcpp::Node> ros_node_;
    double time_window_sec_;
    size_t max_points_;
    int update_rate_ms_;

    bool smooth_curves_enabled_;
    bool minor_grid_enabled_;

    static constexpr double PWM_PERIOD_US = 20000.0;
};
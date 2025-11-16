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
    double arm_angle;          // Ángulo del brazo en grados
    double motor_speed;        // Velocidad del motor en rad/s
    double pwm_input_us;       // PWM input en microsegundos
    double duty_cycle_percent; // Duty cycle calculado en porcentaje

    // Datos de simulación
    double sim_arm_angle;          // Ángulo simulado
    double sim_motor_speed;        // Velocidad simulada
    double sim_pwm_input_us;       // PWM simulado
    double sim_duty_cycle_percent; // Duty cycle simulado
};

class AerospaceDataVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit AerospaceDataVisualizer(std::shared_ptr<rclcpp::Node> node = nullptr, QWidget *parent = nullptr);
    ~AerospaceDataVisualizer();

    // Data reception interface - incluye datos de simulación
    void onDataReceived(double arm_angle, double motor_speed, double pwm_input_us,
                        double sim_arm_angle, double sim_motor_speed, double sim_pwm_input_us);

    // Configuration methods
    void clearData();
    void setTimeWindow(double seconds);
    void updateTheme();

    // Enhanced configuration methods
    void setSmoothCurves(bool enabled);
    void setMinorGridVisible(bool visible);

    // Configuration getters
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
        QColor sim_color; // Color para datos simulados
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

    // UI components
    QGridLayout *main_layout_;
    std::vector<std::unique_ptr<ChartBase>> charts_;
    std::vector<ChartSetup> chart_setups_;

    // Timing and performance
    QTimer *update_timer_;
    QMutex *update_mutex_;
    double last_update_time_;

    // Configuration
    std::shared_ptr<rclcpp::Node> ros_node_;
    double time_window_sec_;
    size_t max_points_;
    int update_rate_ms_;

    // Enhanced configuration options
    bool smooth_curves_enabled_;
    bool minor_grid_enabled_;

    // PWM configuration constants
    static constexpr double PWM_PERIOD_US = 20000.0;
};
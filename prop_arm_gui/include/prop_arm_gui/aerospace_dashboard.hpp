#pragma once

#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QFont>
#include <QFontMetrics>
#include <QPen>
#include <QBrush>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QRect>
#include <QPointF>
#include <QPolygonF>
#include <cmath>

class AerospaceDashboard : public QWidget
{
    Q_OBJECT

public:
    explicit AerospaceDashboard(QWidget *parent = nullptr);
    ~AerospaceDashboard() = default;

    // Data setters
    void setArmAngle(double angle_deg);
    void setTargetAngle(double target_deg);
    void setMotorSpeed(double speed_rad_s);
    void setThrust(double thrust_n);
    void setError(double error_deg);
    void setVEmf(double v_emf);
    void setSystemStatus(const QString &status);
    void setConnectionStatus(bool connected);

    // Appearance
    void setTheme(const QString &theme = "dark");
    void setUpdateRate(int fps);

signals:
    void clicked();
    void doubleClicked();
    void valueChanged(const QString &parameter, double value);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private slots:
    void updateAnimation();

private:
    void setupUI();
    void calculateLayout();

    // Drawing methods
    void drawBackground(QPainter &painter);
    void drawArmIndicator(QPainter &painter);
    void drawAngleGauge(QPainter &painter);
    void drawThrustGauge(QPainter &painter);
    void drawMotorSpeedGauge(QPainter &painter);
    void drawSystemInfo(QPainter &painter);
    void drawStatusIndicators(QPainter &painter);
    void drawGrid(QPainter &painter);
    void drawHorizonLine(QPainter &painter);
    void drawNumericalDisplays(QPainter &painter);

    // Helper drawing methods
    void drawCircularGauge(QPainter &painter, const QRectF &rect,
                           double value, double min_val, double max_val,
                           const QString &title, const QString &units,
                           const QColor &color, double start_angle = -135, double span_angle = 270);
    void drawLinearGauge(QPainter &painter, const QRectF &rect,
                         double value, double min_val, double max_val,
                         const QString &title, const QString &units, const QColor &color);
    void drawArcGauge(QPainter &painter, const QPointF &center, double radius,
                      double value, double min_val, double max_val,
                      double start_angle, double span_angle, const QColor &color);
    void drawNeedle(QPainter &painter, const QPointF &center, double radius,
                    double angle, const QColor &color, double width = 2.0);
    void drawStatusLED(QPainter &painter, const QPointF &position,
                       const QString &label, bool active, const QColor &color);

    // Utility methods
    QColor interpolateColor(const QColor &color1, const QColor &color2, double factor);
    double mapValue(double value, double in_min, double in_max, double out_min, double out_max);
    QString formatValue(double value, int decimals = 1);

    // Data
    double arm_angle_deg_;
    double target_angle_deg_;
    double motor_speed_rad_s_;
    double thrust_n_;
    double error_deg_;
    double v_emf_;
    QString system_status_;
    bool connected_;

    // Animation
    QTimer *animation_timer_;
    double animation_phase_;
    double arm_angle_display_; // Smoothed display value
    double target_angle_display_;
    double motor_speed_display_;
    double thrust_display_;

    // Layout
    QRectF gauge_rect_;
    QRectF arm_indicator_rect_;
    QRectF status_rect_;
    QRectF info_rect_;
    QPointF center_;
    double scale_factor_;

    // Styling
    QString theme_;
    QColor background_color_;
    QColor primary_color_;
    QColor secondary_color_;
    QColor success_color_;
    QColor warning_color_;
    QColor danger_color_;
    QColor text_color_;
    QColor grid_color_;
    QColor gauge_bg_color_;

    // Fonts
    QFont title_font_;
    QFont value_font_;
    QFont label_font_;
    QFont small_font_;

    // Configuration
    static constexpr double MAX_ARM_ANGLE = 90.0;
    static constexpr double MIN_ARM_ANGLE = -90.0;
    static constexpr double MAX_MOTOR_SPEED = 785.0;
    static constexpr double MAX_THRUST = 45.0;
    static constexpr double MAX_V_EMF = 20.0;
    static constexpr int DEFAULT_FPS = 30;
    static constexpr double SMOOTHING_FACTOR = 0.15;

    // Animation constants
    static constexpr double PI = 3.14159265359;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;
};
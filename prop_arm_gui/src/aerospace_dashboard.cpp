#include "prop_arm_gui/aerospace_dashboard.hpp"
#include <QPainter>
#include <QResizeEvent>
#include <QMouseEvent>
#include <algorithm>

AerospaceDashboard::AerospaceDashboard(QWidget *parent)
    : QWidget(parent),
      arm_angle_deg_(0.0), target_angle_deg_(0.0), motor_speed_rad_s_(0.0),
      thrust_n_(0.0), error_deg_(0.0), v_emf_(0.0),
      system_status_("Initializing"), connected_(false),
      animation_phase_(0.0), arm_angle_display_(0.0), target_angle_display_(0.0),
      motor_speed_display_(0.0), thrust_display_(0.0), scale_factor_(1.0)
{
    setupUI();

    // Animation timer
    animation_timer_ = new QTimer(this);
    connect(animation_timer_, &QTimer::timeout, this, &AerospaceDashboard::updateAnimation);
    animation_timer_->start(1000 / DEFAULT_FPS);

    setMinimumSize(400, 300);
    setMouseTracking(true);
}

void AerospaceDashboard::setupUI()
{
    setTheme("dark");

    // Initialize fonts
    title_font_ = QFont("Arial", 12, QFont::Bold);
    value_font_ = QFont("Arial", 14, QFont::Bold);
    label_font_ = QFont("Arial", 10);
    small_font_ = QFont("Arial", 8);
}

void AerospaceDashboard::setTheme(const QString &theme)
{
    theme_ = theme;

    if (theme == "dark")
    {
        background_color_ = QColor(15, 23, 42);  // Dark slate
        primary_color_ = QColor(30, 58, 138);    // Deep blue
        secondary_color_ = QColor(59, 130, 246); // Lighter blue
        success_color_ = QColor(16, 185, 129);   // Green
        warning_color_ = QColor(245, 158, 11);   // Orange
        danger_color_ = QColor(239, 68, 68);     // Red
        text_color_ = QColor(241, 245, 249);     // Light text
        grid_color_ = QColor(64, 128, 255, 50);  // Semi-transparent blue
        gauge_bg_color_ = QColor(30, 41, 59);    // Slate 800
    }
    update();
}

void AerospaceDashboard::calculateLayout()
{
    QRect rect = this->rect();
    center_ = rect.center();
    scale_factor_ = std::min(rect.width(), rect.height()) / 400.0;

    // Calculate layout rectangles
    int margin = 20 * scale_factor_;
    arm_indicator_rect_ = QRectF(center_.x() - 150 * scale_factor_,
                                 center_.y() - 150 * scale_factor_,
                                 300 * scale_factor_, 300 * scale_factor_);

    status_rect_ = QRectF(margin, margin, 200 * scale_factor_, 100 * scale_factor_);
    info_rect_ = QRectF(rect.width() - 200 * scale_factor_ - margin, margin,
                        200 * scale_factor_, 100 * scale_factor_);
}

void AerospaceDashboard::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    calculateLayout();

    drawBackground(painter);
    drawGrid(painter);
    drawHorizonLine(painter);
    drawArmIndicator(painter);
    drawAngleGauge(painter);
    drawThrustGauge(painter);
    drawMotorSpeedGauge(painter);
    drawSystemInfo(painter);
    drawStatusIndicators(painter);
    drawNumericalDisplays(painter);
}

void AerospaceDashboard::drawBackground(QPainter &painter)
{
    // Fill background
    painter.fillRect(rect(), background_color_);

    // Draw subtle gradient
    QLinearGradient gradient(0, 0, 0, height());
    gradient.setColorAt(0, background_color_.lighter(110));
    gradient.setColorAt(1, background_color_.darker(110));
    painter.fillRect(rect(), gradient);
}

void AerospaceDashboard::drawGrid(QPainter &painter)
{
    painter.setPen(QPen(grid_color_, 1));

    int gridSize = 40 * scale_factor_;

    // Vertical lines
    for (int x = gridSize; x < width(); x += gridSize)
    {
        painter.drawLine(x, 0, x, height());
    }

    // Horizontal lines
    for (int y = gridSize; y < height(); y += gridSize)
    {
        painter.drawLine(0, y, width(), y);
    }
}

void AerospaceDashboard::drawHorizonLine(QPainter &painter)
{
    painter.setPen(QPen(secondary_color_, 2 * scale_factor_));
    painter.drawLine(0, center_.y(), width(), center_.y());

    // Draw horizon markers
    painter.setPen(QPen(text_color_, 1));
    painter.setFont(small_font_);
    painter.drawText(10, center_.y() - 5, "0°");
}

void AerospaceDashboard::drawArmIndicator(QPainter &painter)
{
    painter.save();

    // Draw outer circle
    painter.setPen(QPen(primary_color_, 3 * scale_factor_));
    painter.setBrush(QBrush(gauge_bg_color_));
    painter.drawEllipse(arm_indicator_rect_);

    // Draw center point
    QPointF armCenter = arm_indicator_rect_.center();
    painter.setBrush(QBrush(text_color_));
    painter.drawEllipse(armCenter, 5 * scale_factor_, 5 * scale_factor_);

    // Draw current arm position
    double armRadius = arm_indicator_rect_.width() / 2 - 20 * scale_factor_;
    drawNeedle(painter, armCenter, armRadius,
               -arm_angle_display_ - 90, success_color_, 4 * scale_factor_);

    // Draw target position (if different)
    if (std::abs(target_angle_display_ - arm_angle_display_) > 1.0)
    {
        drawNeedle(painter, armCenter, armRadius * 0.8,
                   -target_angle_display_ - 90, warning_color_, 2 * scale_factor_);
    }

    // Draw angle markings
    painter.setPen(QPen(text_color_, 1));
    painter.setFont(small_font_);

    for (int angle = -90; angle <= 90; angle += 30)
    {
        double rad = (angle - 90) * DEG_TO_RAD;
        QPointF outer(armCenter.x() + (armRadius + 10) * cos(rad),
                      armCenter.y() + (armRadius + 10) * sin(rad));
        QPointF inner(armCenter.x() + (armRadius - 5) * cos(rad),
                      armCenter.y() + (armRadius - 5) * sin(rad));

        painter.setPen(QPen(grid_color_, 2));
        painter.drawLine(inner, outer);

        painter.setPen(QPen(text_color_, 1));
        QString angleText = QString::number(-angle) + "°";
        QRect textRect(outer.x() - 20, outer.y() - 10, 40, 20);
        painter.drawText(textRect, Qt::AlignCenter, angleText);
    }

    painter.restore();
}

void AerospaceDashboard::drawAngleGauge(QPainter &painter)
{
    QRectF gaugeRect(20 * scale_factor_, height() - 120 * scale_factor_,
                     200 * scale_factor_, 80 * scale_factor_);

    drawLinearGauge(painter, gaugeRect, arm_angle_display_,
                    MIN_ARM_ANGLE, MAX_ARM_ANGLE, "ARM ANGLE", "°", success_color_);
}

void AerospaceDashboard::drawThrustGauge(QPainter &painter)
{
    QRectF gaugeRect(width() - 220 * scale_factor_, height() - 120 * scale_factor_,
                     200 * scale_factor_, 80 * scale_factor_);

    drawLinearGauge(painter, gaugeRect, thrust_display_,
                    0, MAX_THRUST, "THRUST", "N", danger_color_);
}

void AerospaceDashboard::drawMotorSpeedGauge(QPainter &painter)
{
    QRectF gaugeRect(20 * scale_factor_, height() - 220 * scale_factor_,
                     120 * scale_factor_, 120 * scale_factor_);

    drawCircularGauge(painter, gaugeRect, motor_speed_display_,
                      0, MAX_MOTOR_SPEED, "MOTOR", "rad/s", secondary_color_);
}

void AerospaceDashboard::drawLinearGauge(QPainter &painter, const QRectF &rect,
                                         double value, double min_val, double max_val,
                                         const QString &title, const QString &units,
                                         const QColor &color)
{
    painter.save();

    // Background
    painter.setPen(QPen(primary_color_, 2));
    painter.setBrush(QBrush(gauge_bg_color_));
    painter.drawRoundedRect(rect, 5, 5);

    // Title
    painter.setPen(QPen(text_color_));
    painter.setFont(label_font_);
    QRectF titleRect(rect.x(), rect.y(), rect.width(), 20);
    painter.drawText(titleRect, Qt::AlignCenter, title);

    // Value bar background
    QRectF barRect(rect.x() + 10, rect.y() + 25, rect.width() - 20, 20);
    painter.setPen(QPen(grid_color_, 1));
    painter.setBrush(QBrush(background_color_));
    painter.drawRoundedRect(barRect, 3, 3);

    // Value bar
    double normalizedValue = (value - min_val) / (max_val - min_val);
    normalizedValue = std::clamp(normalizedValue, 0.0, 1.0);

    QRectF valueRect(barRect.x(), barRect.y(),
                     barRect.width() * normalizedValue, barRect.height());

    QLinearGradient gradient(valueRect.topLeft(), valueRect.bottomRight());
    gradient.setColorAt(0, color.lighter(120));
    gradient.setColorAt(1, color);

    painter.setBrush(QBrush(gradient));
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(valueRect, 3, 3);

    // Value text
    painter.setPen(QPen(text_color_));
    painter.setFont(value_font_);
    QString valueText = formatValue(value, 1) + " " + units;
    QRectF valueTextRect(rect.x(), rect.y() + 50, rect.width(), 20);
    painter.drawText(valueTextRect, Qt::AlignCenter, valueText);

    painter.restore();
}

void AerospaceDashboard::drawCircularGauge(QPainter &painter, const QRectF &rect,
                                           double value, double min_val, double max_val,
                                           const QString &title, const QString &units,
                                           const QColor &color, double start_angle, double span_angle)
{
    painter.save();

    QPointF center = rect.center();
    double radius = std::min(rect.width(), rect.height()) / 2 - 10;

    // Background arc
    painter.setPen(QPen(gauge_bg_color_, 8 * scale_factor_));
    painter.drawArc(QRectF(center.x() - radius, center.y() - radius,
                           2 * radius, 2 * radius),
                    start_angle * 16, span_angle * 16);

    // Value arc
    double normalizedValue = (value - min_val) / (max_val - min_val);
    normalizedValue = std::clamp(normalizedValue, 0.0, 1.0);

    painter.setPen(QPen(color, 8 * scale_factor_, Qt::SolidLine, Qt::RoundCap));
    painter.drawArc(QRectF(center.x() - radius, center.y() - radius,
                           2 * radius, 2 * radius),
                    start_angle * 16, span_angle * normalizedValue * 16);

    // Center text
    painter.setPen(QPen(text_color_));
    painter.setFont(label_font_);
    QRectF titleRect(center.x() - 40, center.y() - 30, 80, 15);
    painter.drawText(titleRect, Qt::AlignCenter, title);

    painter.setFont(value_font_);
    QString valueText = formatValue(value, 0);
    QRectF valueRect(center.x() - 40, center.y() - 10, 80, 20);
    painter.drawText(valueRect, Qt::AlignCenter, valueText);

    painter.setFont(small_font_);
    QRectF unitsRect(center.x() - 40, center.y() + 10, 80, 15);
    painter.drawText(unitsRect, Qt::AlignCenter, units);

    painter.restore();
}

void AerospaceDashboard::drawNeedle(QPainter &painter, const QPointF &center, double radius,
                                    double angle, const QColor &color, double width)
{
    painter.save();

    double rad = angle * DEG_TO_RAD;
    QPointF tip(center.x() + radius * cos(rad), center.y() + radius * sin(rad));
    QPointF base1(center.x() - 10 * cos(rad + PI / 2), center.y() - 10 * sin(rad + PI / 2));
    QPointF base2(center.x() - 10 * cos(rad - PI / 2), center.y() - 10 * sin(rad - PI / 2));

    QPolygonF needle;
    needle << tip << base1 << center << base2;

    // Use the width parameter
    painter.setPen(QPen(color.darker(120), width));
    painter.setBrush(QBrush(color));
    painter.drawPolygon(needle);

    painter.restore();
}

void AerospaceDashboard::drawSystemInfo(QPainter &painter)
{
    painter.save();

    // System info panel
    painter.setPen(QPen(primary_color_, 2));
    painter.setBrush(QBrush(gauge_bg_color_));
    painter.drawRoundedRect(status_rect_, 5, 5);

    // Title
    painter.setPen(QPen(text_color_));
    painter.setFont(title_font_);
    QRectF titleRect(status_rect_.x() + 10, status_rect_.y() + 5,
                     status_rect_.width() - 20, 20);
    painter.drawText(titleRect, Qt::AlignLeft, "SYSTEM STATUS");

    // Status text
    painter.setFont(label_font_);
    QRectF statusTextRect(status_rect_.x() + 10, status_rect_.y() + 30,
                          status_rect_.width() - 20, 20);
    painter.drawText(statusTextRect, Qt::AlignLeft, system_status_);

    // Error display
    if (std::abs(error_deg_) > 0.1)
    {
        QColor errorColor = std::abs(error_deg_) > 5.0 ? danger_color_ : warning_color_;
        painter.setPen(QPen(errorColor));
        QRectF errorRect(status_rect_.x() + 10, status_rect_.y() + 55,
                         status_rect_.width() - 20, 20);
        QString errorText = QString("ERROR: %1°").arg(formatValue(error_deg_, 2));
        painter.drawText(errorRect, Qt::AlignLeft, errorText);
    }

    painter.restore();
}

void AerospaceDashboard::drawStatusIndicators(QPainter &painter)
{
    painter.save();

    // Connection indicator
    QPointF connPos(width() - 50 * scale_factor_, 30 * scale_factor_);
    drawStatusLED(painter, connPos, "CONN", connected_, success_color_);

    // System status indicators
    QPointF sysPos(width() - 50 * scale_factor_, 60 * scale_factor_);
    bool systemOK = (system_status_ == "Active" || system_status_ == "Ready");
    drawStatusLED(painter, sysPos, "SYS", systemOK, success_color_);

    painter.restore();
}

void AerospaceDashboard::drawStatusLED(QPainter &painter, const QPointF &position,
                                       const QString &label, bool active, const QColor &color)
{
    painter.save();

    // LED circle
    QColor ledColor = active ? color : color.darker(300);
    painter.setPen(QPen(ledColor.lighter(150), 2));
    painter.setBrush(QBrush(ledColor));

    QRectF ledRect(position.x() - 8, position.y() - 8, 16, 16);
    painter.drawEllipse(ledRect);

    // LED glow effect
    if (active)
    {
        QRadialGradient glow(position, 12);
        glow.setColorAt(0, color.lighter(150));
        glow.setColorAt(0.5, color);
        glow.setColorAt(1, QColor(0, 0, 0, 0));
        painter.setBrush(QBrush(glow));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QRectF(position.x() - 12, position.y() - 12, 24, 24));
    }

    // Label
    painter.setPen(QPen(text_color_));
    painter.setFont(small_font_);
    QRectF labelRect(position.x() - 20, position.y() + 15, 40, 15);
    painter.drawText(labelRect, Qt::AlignCenter, label);

    painter.restore();
}

void AerospaceDashboard::drawNumericalDisplays(QPainter &painter)
{
    painter.save();

    // Numerical display panel
    painter.setPen(QPen(primary_color_, 2));
    painter.setBrush(QBrush(gauge_bg_color_));
    painter.drawRoundedRect(info_rect_, 5, 5);

    painter.setPen(QPen(text_color_));
    painter.setFont(label_font_);

    // Title
    QRectF titleRect(info_rect_.x() + 10, info_rect_.y() + 5,
                     info_rect_.width() - 20, 15);
    painter.drawText(titleRect, Qt::AlignLeft, "TELEMETRY");

    // Values
    QStringList labels = {"V_EMF:", "TARGET:", "ACTUAL:"};
    QStringList values = {
        formatValue(v_emf_, 3) + "V",
        formatValue(target_angle_deg_, 1) + "°",
        formatValue(arm_angle_deg_, 1) + "°"};

    for (int i = 0; i < labels.size(); ++i)
    {
        QRectF labelRect(info_rect_.x() + 10, info_rect_.y() + 25 + i * 20, 60, 15);
        QRectF valueRect(info_rect_.x() + 70, info_rect_.y() + 25 + i * 20,
                         info_rect_.width() - 80, 15);

        painter.drawText(labelRect, Qt::AlignLeft, labels[i]);
        painter.setFont(value_font_);
        painter.drawText(valueRect, Qt::AlignRight, values[i]);
        painter.setFont(label_font_);
    }

    painter.restore();
}

void AerospaceDashboard::updateAnimation()
{
    animation_phase_ += 0.1;
    if (animation_phase_ > 2 * PI)
    {
        animation_phase_ = 0;
    }

    // Smooth value transitions
    arm_angle_display_ += (arm_angle_deg_ - arm_angle_display_) * SMOOTHING_FACTOR;
    target_angle_display_ += (target_angle_deg_ - target_angle_display_) * SMOOTHING_FACTOR;
    motor_speed_display_ += (motor_speed_rad_s_ - motor_speed_display_) * SMOOTHING_FACTOR;
    thrust_display_ += (thrust_n_ - thrust_display_) * SMOOTHING_FACTOR;

    update();
}

void AerospaceDashboard::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    calculateLayout();
}

void AerospaceDashboard::mousePressEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    emit clicked();
}

void AerospaceDashboard::mouseDoubleClickEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    emit doubleClicked();
}

// Setters
void AerospaceDashboard::setArmAngle(double angle_deg)
{
    if (std::abs(arm_angle_deg_ - angle_deg) > 0.01)
    {
        arm_angle_deg_ = angle_deg;
        emit valueChanged("arm_angle", angle_deg);
    }
}

void AerospaceDashboard::setTargetAngle(double target_deg)
{
    if (std::abs(target_angle_deg_ - target_deg) > 0.01)
    {
        target_angle_deg_ = target_deg;
        emit valueChanged("target_angle", target_deg);
    }
}

void AerospaceDashboard::setMotorSpeed(double speed_rad_s)
{
    if (std::abs(motor_speed_rad_s_ - speed_rad_s) > 0.1)
    {
        motor_speed_rad_s_ = speed_rad_s;
        emit valueChanged("motor_speed", speed_rad_s);
    }
}

void AerospaceDashboard::setThrust(double thrust_n)
{
    if (std::abs(thrust_n_ - thrust_n) > 0.01)
    {
        thrust_n_ = thrust_n;
        emit valueChanged("thrust", thrust_n);
    }
}

void AerospaceDashboard::setError(double error_deg)
{
    if (std::abs(error_deg_ - error_deg) > 0.01)
    {
        error_deg_ = error_deg;
        emit valueChanged("error", error_deg);
    }
}

void AerospaceDashboard::setVEmf(double v_emf)
{
    if (std::abs(v_emf_ - v_emf) > 0.001)
    {
        v_emf_ = v_emf;
        emit valueChanged("v_emf", v_emf);
    }
}

void AerospaceDashboard::setSystemStatus(const QString &status)
{
    if (system_status_ != status)
    {
        system_status_ = status;
    }
}

void AerospaceDashboard::setConnectionStatus(bool connected)
{
    if (connected_ != connected)
    {
        connected_ = connected;
    }
}

void AerospaceDashboard::setUpdateRate(int fps)
{
    if (fps > 0 && fps <= 60)
    {
        animation_timer_->stop();
        animation_timer_->start(1000 / fps);
    }
}

// Utility methods
QColor AerospaceDashboard::interpolateColor(const QColor &color1, const QColor &color2, double factor)
{
    factor = std::clamp(factor, 0.0, 1.0);

    int r = color1.red() + factor * (color2.red() - color1.red());
    int g = color1.green() + factor * (color2.green() - color1.green());
    int b = color1.blue() + factor * (color2.blue() - color1.blue());
    int a = color1.alpha() + factor * (color2.alpha() - color1.alpha());

    return QColor(r, g, b, a);
}

double AerospaceDashboard::mapValue(double value, double in_min, double in_max,
                                    double out_min, double out_max)
{
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

QString AerospaceDashboard::formatValue(double value, int decimals)
{
    return QString::number(value, 'f', decimals);
}
#include "prop_arm_gui/ui/widgets/arm_visualizer.hpp"

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QFile>
#include <QImage>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace prop_arm::ui::widgets {

ArmVisualizer::ArmVisualizer(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(250, 250);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    loadSvgModels_();

    // Manual size adjustments
    arm_scale_factor_ = 0.25;
    base_scale_factor_ = 0.4;
    pivot_y_offset_ = 0.4;

    // Manual center offsets
    arm_center_x_ = 0.5;
    arm_center_y_ = 0.5;
    base_center_x_ = 0.5;
    base_center_y_ = 0.5;

    // Angle offset in RADIANS
    arm_angle_offset_ = -3.14159;

    // Opacity adjustments
    ref_opacity_ = 1.0;
    real_opacity_ = 1.0;
    sim_opacity_ = 1.0;
}

bool ArmVisualizer::loadSvgModels_() {
    QString arm_path = ":/models/arm_segment.svg";
    if (QFile::exists(arm_path)) {
        arm_renderer_ = std::make_unique<QSvgRenderer>(arm_path);
        if (arm_renderer_->isValid()) {
            arm_svg_size_ = arm_renderer_->defaultSize();
        } else {
            arm_renderer_.reset();
        }
    }

    QString base_path = ":/models/base_segment.svg";
    if (QFile::exists(base_path)) {
        base_renderer_ = std::make_unique<QSvgRenderer>(base_path);
        if (base_renderer_->isValid()) {
            base_svg_size_ = base_renderer_->defaultSize();
        } else {
            base_renderer_.reset();
        }
    }

    return arm_renderer_ && base_renderer_;
}

void ArmVisualizer::setRefAngle(double rad) {
    if (!std::isfinite(rad)) return;
    ref_angle_ = rad;
    update();
}

void ArmVisualizer::setRealAngle(double rad) {
    if (!std::isfinite(rad)) return;
    real_angle_ = rad;
    update();
}

void ArmVisualizer::setSimAngle(double rad) {
    if (!std::isfinite(rad)) return;
    sim_angle_ = rad;
    update();
}

void ArmVisualizer::setShowRef(bool show) {
    if (show_ref_ == show) return;
    show_ref_ = show;
    update();
}

void ArmVisualizer::setShowReal(bool show) {
    if (show_real_ == show) return;
    show_real_ = show;
    update();
}

void ArmVisualizer::setShowSim(bool show) {
    if (show_sim_ == show) return;
    show_sim_ = show;
    update();
}

void ArmVisualizer::applyTheme(const prop_arm::ui::style::ThemeSpec& spec) {
    // Use theme colors for ref, real, and sim
    ref_color_ = spec.ref_color;    // NEW: from theme
    real_color_ = spec.accent;
    sim_color_ = spec.accent2;
    base_color_ = spec.text_muted;
    bg_color_ = spec.panel;
    update();
}

void ArmVisualizer::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    painter.fillRect(rect(), bg_color_);

    const double w = width();
    const double h = height();
    const QPointF pivot(w / 2.0, h * pivot_y_offset_);

    if (show_sim_) {
        drawArm_(painter, pivot, sim_angle_, sim_color_, true, sim_opacity_);
    }
    if (show_real_) {
        drawArm_(painter, pivot, real_angle_, real_color_, true, real_opacity_);
    }
    if (show_ref_) {
        drawArm_(painter, pivot, ref_angle_, ref_color_, false, ref_opacity_);
    }

    drawBase_(painter, pivot);

    // Labels
    QFont font = painter.font();
    font.setPointSize(9);
    painter.setFont(font);

    int y = static_cast<int>(h * 0.88);

    if (show_ref_) {
        painter.setPen(ref_color_);
        painter.drawText(10, y, QString("Ref: %1°").arg(ref_angle_ * 180.0 / M_PI, 0, 'f', 1));
        y += 16;
    }
    if (show_real_) {
        painter.setPen(real_color_);
        painter.drawText(10, y, QString("Real: %1°").arg(real_angle_ * 180.0 / M_PI, 0, 'f', 1));
        y += 16;
    }
    if (show_sim_) {
        painter.setPen(sim_color_);
        painter.drawText(10, y, QString("Sim: %1°").arg(sim_angle_ * 180.0 / M_PI, 0, 'f', 1));
    }
}

void ArmVisualizer::drawBase_(QPainter& painter, const QPointF& pivot) {
    const double scale = std::min(width(), height()) / 300.0;

    if (base_renderer_ && base_renderer_->isValid()) {
        const double bw = base_svg_size_.width() * scale * base_scale_factor_;
        const double bh = base_svg_size_.height() * scale * base_scale_factor_;

        const double offset_x = bw * base_center_x_;
        const double offset_y = bh * base_center_y_;

        QRectF base_rect(pivot.x() - offset_x, pivot.y() - offset_y, bw, bh);

        painter.save();
        painter.setOpacity(0.9);
        base_renderer_->render(&painter, base_rect);
        painter.restore();

        painter.setBrush(base_color_.darker(150));
        painter.setPen(Qt::NoPen);
        const double pivot_size = 6 * scale * base_scale_factor_;
        painter.drawEllipse(pivot, pivot_size, pivot_size);

    } else {
        const double bw = base_width_ * scale * base_scale_factor_;
        const double bh = base_height_ * scale * base_scale_factor_;

        QRectF base_rect(pivot.x() - bw / 2.0, pivot.y() - bh / 2.0, bw, bh);

        painter.setPen(QPen(base_color_.darker(130), 2));
        painter.setBrush(base_color_);
        painter.drawRoundedRect(base_rect, 4 * scale, 4 * scale);

        painter.setBrush(base_color_.darker(150));
        painter.setPen(Qt::NoPen);

        const double pivot_size = 6 * scale * base_scale_factor_;
        painter.drawEllipse(pivot, pivot_size, pivot_size);
    }
}

void ArmVisualizer::drawArm_(QPainter& painter,
                             const QPointF& pivot,
                             double angle_rad,
                             const QColor& color,
                             bool shadow,
                             double opacity) {
    const double scale = std::min(width(), height()) / 300.0;

    painter.save();
    painter.translate(pivot);

    const double corrected_angle = angle_rad + arm_angle_offset_;
    const double deg = -(corrected_angle * 180.0 / M_PI) - 90.0;
    painter.rotate(deg);

    if (arm_renderer_ && arm_renderer_->isValid()) {
        const double aw = arm_svg_size_.width() * scale * arm_scale_factor_;
        const double ah = arm_svg_size_.height() * scale * arm_scale_factor_;

        const double offset_x = aw * arm_center_x_;
        const double offset_y = ah * arm_center_y_;

        QRectF arm_rect(-offset_x, -offset_y, aw, ah);

        QImage svg_image(arm_rect.size().toSize(), QImage::Format_ARGB32_Premultiplied);
        svg_image.fill(Qt::transparent);

        QPainter svg_painter(&svg_image);
        svg_painter.setRenderHint(QPainter::Antialiasing, true);
        svg_painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

        arm_renderer_->render(&svg_painter, QRectF(QPointF(0, 0), arm_rect.size()));
        svg_painter.end();

        opacity = std::clamp(opacity, 0.0, 1.0);
        const int alpha_value = static_cast<int>(opacity * 255);

        for (int y = 0; y < svg_image.height(); ++y) {
            QRgb* line = reinterpret_cast<QRgb*>(svg_image.scanLine(y));
            for (int x = 0; x < svg_image.width(); ++x) {
                QRgb pixel = line[x];
                int alpha = qAlpha(pixel);

                if (alpha > 0) {
                    int gray = qGray(pixel);

                    if (gray < 50) {
                        line[x] = qRgba(color.red(), color.green(), color.blue(), alpha_value);
                    } else {
                        int adjusted_alpha = static_cast<int>((alpha * opacity * 255) / 255);
                        line[x] = qRgba(qRed(pixel), qGreen(pixel), qBlue(pixel), adjusted_alpha);
                    }
                }
            }
        }

        painter.drawImage(arm_rect, svg_image);

    } else {
        const double length = arm_length_ * scale * arm_scale_factor_;
        const double aw = arm_width_ * scale * arm_scale_factor_;
        const double half_w = aw / 2.0;
        QRectF arm_rect(0, -half_w, length, aw);

        opacity = std::clamp(opacity, 0.0, 1.0);
        const int alpha_value = static_cast<int>(opacity * 255);

        if (shadow) {
            QColor shadow_fill = color;
            shadow_fill.setAlpha(alpha_value);

            painter.setPen(Qt::NoPen);
            painter.setBrush(shadow_fill);
            painter.drawRoundedRect(arm_rect, 3 * scale, 3 * scale);

        } else {
            QColor ref_fill = color;
            ref_fill.setAlpha(alpha_value);

            painter.setPen(Qt::NoPen);
            painter.setBrush(ref_fill);
            painter.drawRoundedRect(arm_rect, 3 * scale, 3 * scale);

            if (opacity > 0.5) {
                QColor highlight = color.lighter(140);
                highlight.setAlpha(static_cast<int>(80 * opacity));
                painter.setBrush(highlight);

                QRectF hi_rect(length * 0.15, -half_w * 0.5, length * 0.15, aw * 0.5);
                painter.drawRoundedRect(hi_rect, 2 * scale, 2 * scale);
            }
        }
    }

    painter.restore();
}

}  // namespace prop_arm::ui::widgets
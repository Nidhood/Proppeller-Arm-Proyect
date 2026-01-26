#pragma once
#include <QWidget>
#include <QColor>
#include <QSvgRenderer>
#include <QSize>
#include <memory>
#include "prop_arm_gui/ui/style/theme.hpp"

namespace prop_arm::ui::widgets {

class ArmVisualizer final : public QWidget {
    Q_OBJECT

public:
    explicit ArmVisualizer(QWidget* parent = nullptr);

    void setRefAngle(double rad);
    void setRealAngle(double rad);
    void setSimAngle(double rad);

    void setShowRef(bool show);
    void setShowReal(bool show);
    void setShowSim(bool show);

    void applyTheme(const prop_arm::ui::style::ThemeSpec& spec);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawBase_(QPainter& painter, const QPointF& pivot);
    void drawArm_(QPainter& painter,
                  const QPointF& pivot,
                  double angle_rad,
                  const QColor& color,
                  bool shadow,
                  double opacity);
    bool loadSvgModels_();

private:
    // SVG renderers
    std::unique_ptr<QSvgRenderer> arm_renderer_;
    std::unique_ptr<QSvgRenderer> base_renderer_;

    // SVG dimensions (actual size from files)
    QSize arm_svg_size_;
    QSize base_svg_size_;

    // Angles (rad)
    double ref_angle_{0.0};
    double real_angle_{0.0};
    double sim_angle_{0.0};

    // IMPORTANT: default false (avoid fake signals)
    bool show_ref_{false};
    bool show_real_{false};
    bool show_sim_{false};

    // Theme colors (now all set by theme)
    QColor ref_color_{100, 255, 100};
    QColor real_color_{255, 100, 100};
    QColor sim_color_{100, 150, 255};
    QColor base_color_{150, 150, 150};
    QColor bg_color_{30, 30, 35};

    // Fallback dimensions (pixels, scaled) - only used if SVG fails to load
    double base_width_{60.0};
    double base_height_{40.0};
    double arm_length_{120.0};
    double arm_width_{25.0};

    // Manual size and position adjustments
    double arm_scale_factor_{1.0};
    double base_scale_factor_{1.0};
    double pivot_y_offset_{0.6};

    // SVG center offsets (relative to SVG dimensions, 0.5 = center)
    double arm_center_x_{0.0};
    double arm_center_y_{0.5};
    double base_center_x_{0.5};
    double base_center_y_{0.5};

    // Angle offset (radians)
    double arm_angle_offset_{0.0};

    // Opacity controls (0.0 = fully transparent, 1.0 = fully opaque)
    double ref_opacity_{1.0};
    double real_opacity_{0.85};
    double sim_opacity_{0.85};
};

}  // namespace prop_arm::ui::widgets
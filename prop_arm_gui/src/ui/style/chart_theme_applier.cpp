#include "prop_arm_gui/ui/style/chart_theme_applier.hpp"

#include <QBrush>
#include <QFont>
#include <QLegendMarker>
#include <QPen>
#include <QLayout>
#include <QMargins>
#include <QGraphicsLayout>

namespace prop_arm::ui::style {

void ChartThemeApplier::apply(QChart& chart,
                              QLegend* legend,
                              QValueAxis& x,
                              QValueAxis& y,
                              QLineSeries* real,
                              QLineSeries* sim,
                              QLineSeries* ref,
                              const ThemeSpec& spec,
                              bool show_sim,
                              bool show_ref,
                              bool real_live,
                              bool sim_live,
                              bool ref_live,
                              bool integer_y_axis) const {
    chart.setBackgroundRoundness(0);
    chart.setMargins(QMargins(16, 16, 16, 16));
    if (auto* lay = chart.layout()) {
        lay->setContentsMargins(0, 0, 0, 0);
    }
    chart.setBackgroundPen(Qt::NoPen);
    chart.setBackgroundBrush(QBrush(spec.panel));
    chart.setPlotAreaBackgroundVisible(true);
    chart.setPlotAreaBackgroundBrush(QBrush(spec.bg));

    // Title
    QFont title_font = chart.titleFont();
    title_font.setBold(true);
    title_font.setPointSize(17);
    chart.setTitleFont(title_font);
    chart.setTitleBrush(QBrush(spec.text));

    // Legend
    if (legend) {
        legend->setBackgroundVisible(false);
        legend->setBorderColor(Qt::transparent);
        legend->setLabelColor(spec.text);

        QFont legend_font;
        legend_font.setPointSize(11);
        legend->setFont(legend_font);
        legend->setMarkerShape(QLegend::MarkerShapeRectangle);
    }

    // Axis fonts
    QFont axis_label_font;
    axis_label_font.setPointSize(11);
    x.setLabelsFont(axis_label_font);
    y.setLabelsFont(axis_label_font);
    x.setLabelsColor(spec.text);
    y.setLabelsColor(spec.text);

    QFont axis_title_font;
    axis_title_font.setPointSize(12);
    axis_title_font.setBold(true);
    x.setTitleFont(axis_title_font);
    y.setTitleFont(axis_title_font);
    x.setTitleBrush(QBrush(spec.text));
    y.setTitleBrush(QBrush(spec.text));

    QColor grid = spec.text_muted;
    grid.setAlpha(70);
    x.setGridLineColor(grid);
    y.setGridLineColor(grid);

    QColor minor_grid = spec.text_muted;
    minor_grid.setAlpha(40);
    x.setMinorGridLineColor(minor_grid);
    y.setMinorGridLineColor(minor_grid);
    x.setMinorGridLineVisible(false);
    y.setMinorGridLineVisible(false);

    QColor axis_color = spec.text_muted;
    axis_color.setAlpha(120);
    x.setLinePen(QPen(axis_color));
    y.setLinePen(QPen(axis_color));

    if (integer_y_axis) {
        y.setLabelFormat("%d");
    } else {
        y.setLabelFormat("%.2f");
    }

    // Real series - uses accent color
    if (real) {
        QPen pr(spec.accent);
        pr.setWidth(2);
        pr.setCapStyle(Qt::RoundCap);
        pr.setJoinStyle(Qt::RoundJoin);
        real->setPen(pr);
    }

    // Sim series - uses accent2 color
    if (sim) {
        QPen ps(spec.accent2);
        ps.setWidth(2);
        ps.setStyle(Qt::DashLine);
        ps.setCapStyle(Qt::RoundCap);
        ps.setJoinStyle(Qt::RoundJoin);
        sim->setPen(ps);
        sim->setVisible(show_sim);
    }

    // Reference series - NOW USES THEME'S ref_color
    if (ref) {
        QPen pf(spec.ref_color);
        pf.setWidth(2);
        pf.setStyle(Qt::DashDotLine);
        pf.setCapStyle(Qt::RoundCap);
        pf.setJoinStyle(Qt::RoundJoin);
        ref->setPen(pf);
        ref->setVisible(show_ref);
    }

    // Legend visibility
    if (legend) {
        if (real) {
            real->setName(real_live ? "Actual Angle" : "");
            for (auto* m : legend->markers(real)) m->setVisible(real_live);
        }
        if (sim) {
            const bool sim_visible = (show_sim && sim_live);
            sim->setName(sim_visible ? "Sim Angle" : "");
            for (auto* m : legend->markers(sim)) m->setVisible(sim_visible);
        }
        if (ref) {
            const bool ref_visible = (show_ref && ref_live);
            ref->setName(ref_visible ? "Reference" : "");
            for (auto* m : legend->markers(ref)) m->setVisible(ref_visible);
        }
    }
}

} // namespace prop_arm::ui::style
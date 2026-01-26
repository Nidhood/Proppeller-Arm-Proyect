// hover_chart_view.cpp
#include "prop_arm_gui/ui/widgets/hover_chart_view.hpp"

#include <QChart>
#include <QDateTime>
#include <QGraphicsScene>
#include <QMouseEvent>

namespace prop_arm::ui::widgets {

HoverChartView::HoverChartView(QWidget* parent)
    : QChartView(parent) {
    setMouseTracking(true);
    setRenderHint(QPainter::Antialiasing, true);
}

HoverChartView::~HoverChartView() {
    clearHoverElements();
}

QPointF HoverChartView::viewportToChartItem_(const QPoint& viewport_pos) const {
    if (!chart()) return {};
    const QPointF scene_pos = mapToScene(viewport_pos);
    return chart()->mapFromScene(scene_pos);
}

void HoverChartView::mouseMoveEvent(QMouseEvent* event) {
    if (!hover_enabled_ || !chart()) {
        QChartView::mouseMoveEvent(event);
        return;
    }

    // Throttle hover updates to reduce CPU usage.
    const qint64 current_time = QDateTime::currentMSecsSinceEpoch();
    if (current_time - last_hover_time_ < 33) {
        QChartView::mouseMoveEvent(event);
        return;
    }
    last_hover_time_ = current_time;

    const QPoint vp = event->pos();
    const QPointF chart_item_p = viewportToChartItem_(vp);

    // IMPORTANT: plotArea() is in chart-item coordinates (same space as chart_item_p).
    const bool in_plot = chart()->plotArea().contains(chart_item_p);

    if (in_plot) {
        const QPointF value = chart()->mapToValue(chart_item_p);
        emit hoverUpdate(value, vp, true);
    } else {
        // Mouse is outside plot area: request full cleanup (line/points/text).
        emit hoverUpdate(QPointF(), QPoint(), false);
    }

    QChartView::mouseMoveEvent(event);
}

void HoverChartView::leaveEvent(QEvent* event) {
    // Mouse left the widget entirely: request full cleanup.
    emit hoverUpdate(QPointF(), QPoint(), false);
    QChartView::leaveEvent(event);
}

void HoverChartView::clearHoverElements() {
    if (!scene()) return;

    auto removeAndDelete = [this](auto*& item) {
        if (!item) return;
        scene()->removeItem(item);
        delete item;
        item = nullptr;
    };

    removeAndDelete(hover_line_);
    removeAndDelete(hover_point_);
    removeAndDelete(hover_point_sim_);
    removeAndDelete(hover_point_ref_);
    removeAndDelete(hover_text_);
}

} // namespace prop_arm::ui::widgets

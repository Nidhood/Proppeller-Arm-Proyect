// hover_chart_view.hpp
#pragma once

#include <QChartView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>
#include <QPointF>

namespace prop_arm::ui::widgets {

class HoverChartView final : public QChartView {
    Q_OBJECT

public:
    explicit HoverChartView(QWidget* parent = nullptr);
    ~HoverChartView() override;

    void setHoverEnabled(bool enabled) noexcept {
        hover_enabled_ = enabled;
    }
    [[nodiscard]] bool hoverEnabled() const noexcept {
        return hover_enabled_;
    }

    // Removes ALL hover graphics (line, points, text) from the scene.
    void clearHoverElements();

signals:
    // value_pos is valid only when valid==true (mouse inside plot area).
    void hoverUpdate(QPointF value_pos, QPoint viewport_pos, bool valid);

protected:
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    [[nodiscard]] QPointF viewportToChartItem_(const QPoint& viewport_pos) const;

private:
    bool hover_enabled_{true};
    qint64 last_hover_time_{0};

    // Owned by the scene; these are just pointers we manage (create/delete).
    QGraphicsLineItem* hover_line_{nullptr};
    QGraphicsEllipseItem* hover_point_{nullptr};       // Real
    QGraphicsEllipseItem* hover_point_sim_{nullptr};   // Sim
    QGraphicsEllipseItem* hover_point_ref_{nullptr};   // Ref  <-- FIXED: now managed and cleared
    QGraphicsTextItem* hover_text_{nullptr};

    friend class ChartBase;
};

} // namespace prop_arm::ui::widgets

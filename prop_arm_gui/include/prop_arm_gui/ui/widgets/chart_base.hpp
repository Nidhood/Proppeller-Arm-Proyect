#pragma once

#include <QChart>
#include <QLegend>
#include <QLineSeries>
#include <QTimer>
#include <QValueAxis>
#include <QVector>
#include <QWidget>

#include <optional>

#include "prop_arm_gui/core/charts/chart_window.hpp"
#include "prop_arm_gui/core/charts/curve_builder.hpp"
#include "prop_arm_gui/core/charts/sample_rate_estimator.hpp"
#include "prop_arm_gui/core/charts/value_filter.hpp"

#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/ui/style/chart_theme_applier.hpp"
#include "prop_arm_gui/ui/style/theme.hpp"
#include "prop_arm_gui/ui/widgets/hover_chart_view.hpp"

namespace prop_arm::ui::widgets {

class ChartBase final : public QWidget {
    Q_OBJECT

public:
    explicit ChartBase(const QString& title,
                       const QString& y_label,
                       QWidget* parent = nullptr);

    void setTitle(const QString& title);
    void setYLabel(const QString& y_label);
    void setXLabel(const QString& x_label);

    void setShowSim(bool enabled);
    void setShowRef(bool enabled);

    void setWindowSeconds(double seconds);
    void setMaxPoints(int max_points);

    void setAutoRange(bool enabled);
    void setSmoothCurves(bool enabled);

    void setRealLive(bool live);
    void setSimLive(bool live);
    void setRefLive(bool live);

    void setIntegerYAxis(bool enabled);

    void setYRange(double y_min, double y_max);
    void clearYRange();

    void setYBounds(double y_min, double y_max);
    void clearYBounds();

    void clear();

    // Real / Sim samples
    void append(double t_sec, double y, prop_arm::models::Domain domain);

    // Reference samples
    void appendRef(double t_sec, double y);

    // NEW: override legend + hover labels (empty => hidden)
    void setSeriesLabels(const QString& real_label,
                         const QString& sim_label,
                         const QString& ref_label);

    void applyTheme(const prop_arm::ui::style::ThemeSpec& spec);

signals:
    void hoveredTextChanged(const QString& text);
    void dataUpdated();

private slots:
    void onHoverUpdate(QPointF chart_pos, QPoint viewport_pos, bool valid);
    void onUpdateTimer_();

private:
    void setupChart_();
    void rebuildSeries_();
    void updateLegend_();

    void updateSeriesOptimized_();
    void updateAxes_();

    void updateHoverDisplay_();
    void cleanupHoverElements_();

private:
    QChart* chart_{nullptr};
    HoverChartView* view_{nullptr};

    QLineSeries* real_{nullptr};
    QLineSeries* sim_{nullptr};
    QLineSeries* ref_{nullptr};

    QValueAxis* x_{nullptr};
    QValueAxis* y_{nullptr};
    QLegend* legend_{nullptr};

    QString title_;
    QString y_label_;
    QString x_label_{"t (s)"};

    bool show_sim_{true};
    bool show_ref_{false};

    bool auto_range_{false};
    bool smooth_curves_{true};

    bool real_live_{false};
    bool sim_live_{false};
    bool ref_live_{false};

    double window_s_{30.0};
    int max_points_{2200};

    bool integer_y_axis_{false};

    bool y_range_locked_{false};
    double y_min_locked_{-1.0};
    double y_max_locked_{ 1.0};

    bool y_bounds_enabled_{false};
    double y_min_bound_{0.0};
    double y_max_bound_{0.0};

    QVector<QPointF> real_series_buffer_;
    QVector<QPointF> sim_series_buffer_;
    QVector<QPointF> ref_series_buffer_;
    QVector<QPointF> real_spline_buffer_;
    QVector<QPointF> sim_spline_buffer_;
    QVector<QPointF> ref_spline_buffer_;

    QTimer* update_timer_{nullptr};
    bool needs_update_{false};

    bool hover_active_{false};
    QPoint hover_viewport_pos_{};

    prop_arm::ui::style::ThemeSpec theme_{};
    prop_arm::ui::style::ChartThemeApplier theme_applier_{};

    prop_arm::core::charts::ChartWindow window_{};
    prop_arm::core::charts::SampleRateEstimator rate_estimator_{};
    prop_arm::core::charts::ValueFilter value_filter_{};
    prop_arm::core::charts::CurveBuilder curve_{};

    bool auto_points_enabled_{true};
    int auto_points_counter_{0};

    double lp_tau_s_{0.035};
    int segments_per_interval_{3};

    // NEW labels (used for legend + hover text)
    QString real_label_{"Actual"};
    QString sim_label_{"Sim"};
    QString ref_label_{"Ref"};
};

} // namespace prop_arm::ui::widgets

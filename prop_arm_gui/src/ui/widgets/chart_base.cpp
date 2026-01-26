#include "prop_arm_gui/ui/widgets/chart_base.hpp"

#include "prop_arm_gui/core/charts/hover_interpolator.hpp"
#include "prop_arm_gui/ui/style/theme_manager.hpp"

#include <QBrush>
#include <QFont>
#include <QGraphicsScene>
#include <QLegendMarker>
#include <QPen>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace prop_arm::ui::widgets {

namespace {
constexpr double kTiny = 1e-12;

constexpr int kAutoPointsUpdatePeriod = 20;
constexpr int kMinPoints = 200;
constexpr int kMaxPointsCap = 20000;

template <class T>
inline void fastClear(T& v) {
    v.clear();
}

inline int clampi(int v, int lo, int hi) {
    return std::max(lo, std::min(v, hi));
}
} // namespace

ChartBase::ChartBase(const QString& title,
                     const QString& y_label,
                     QWidget* parent)
    : QWidget(parent),
      title_(title),
      y_label_(y_label) {

    window_.setLimits({window_s_, max_points_});

    curve_.setMode(smooth_curves_
                   ? prop_arm::core::charts::CurveBuilder::Mode::MonotoneCubic
                   : prop_arm::core::charts::CurveBuilder::Mode::Raw);
    curve_.setSegmentsPerInterval(segments_per_interval_);

    value_filter_.setTau(lp_tau_s_);

    setupChart_();

    update_timer_ = new QTimer(this);
    update_timer_->setInterval(50);
    connect(update_timer_, &QTimer::timeout, this, &ChartBase::onUpdateTimer_);
    update_timer_->start();

    auto& tm = prop_arm::ui::style::ThemeManager::instance();
    applyTheme(tm.currentSpec());
    connect(&tm,
            &prop_arm::ui::style::ThemeManager::themeChanged,
            this,
    [this](auto) {
        applyTheme(prop_arm::ui::style::ThemeManager::instance().currentSpec());
    });

    auto_points_enabled_ = true;
    auto_points_counter_ = 0;
}

void ChartBase::setupChart_() {
    chart_ = new QChart();
    chart_->setAnimationOptions(QChart::NoAnimation);

    // If you pass "" as title (recommended), no title will be shown.
    chart_->setTitle(title_);

    legend_ = chart_->legend();
    legend_->setVisible(true);
    legend_->setAlignment(Qt::AlignTop);

    x_ = new QValueAxis();
    y_ = new QValueAxis();

    x_->setTitleText(x_label_);
    y_->setTitleText(y_label_);
    x_->setTickCount(6);
    y_->setTickCount(6);
    x_->setMinorTickCount(4);
    y_->setMinorTickCount(4);

    if (integer_y_axis_) y_->setLabelFormat("%d");

    chart_->addAxis(x_, Qt::AlignBottom);
    chart_->addAxis(y_, Qt::AlignLeft);

    view_ = new HoverChartView(this);
    view_->setChart(chart_);
    view_->setObjectName("chartWidget");
    view_->setRenderHint(QPainter::Antialiasing, true);
    view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(view_, 1);

    connect(view_, &HoverChartView::hoverUpdate, this, &ChartBase::onHoverUpdate);

    rebuildSeries_();

    x_->setRange(0.0, window_s_);
    y_->setRange(-1.0, 1.0);

    setShowSim(show_sim_);
    setShowRef(show_ref_);
}

void ChartBase::rebuildSeries_() {
    if (real_) {
        chart_->removeSeries(real_);
        delete real_;
        real_ = nullptr;
    }
    if (sim_)  {
        chart_->removeSeries(sim_);
        delete sim_;
        sim_  = nullptr;
    }
    if (ref_)  {
        chart_->removeSeries(ref_);
        delete ref_;
        ref_  = nullptr;
    }

    real_ = new QLineSeries();
    sim_  = new QLineSeries();
    ref_  = new QLineSeries();

    chart_->addSeries(real_);
    chart_->addSeries(sim_);
    chart_->addSeries(ref_);

    real_->attachAxis(x_);
    real_->attachAxis(y_);
    sim_->attachAxis(x_);
    sim_->attachAxis(y_);
    ref_->attachAxis(x_);
    ref_->attachAxis(y_);

    sim_->setVisible(show_sim_);
    ref_->setVisible(show_ref_);

    needs_update_ = true;
    updateLegend_();
}

void ChartBase::setSeriesLabels(const QString& real_label,
                                const QString& sim_label,
                                const QString& ref_label) {
    real_label_ = real_label;
    sim_label_  = sim_label;
    ref_label_  = ref_label;
    updateLegend_();
    needs_update_ = true;
}

void ChartBase::updateLegend_() {
    if (!legend_) return;

    if (real_) {
        const bool visible = real_live_ && !real_label_.isEmpty();
        real_->setName(visible ? real_label_ : "");
        for (auto* m : legend_->markers(real_)) m->setVisible(visible);
    }

    if (sim_) {
        const bool visible = show_sim_ && sim_live_ && !sim_label_.isEmpty();
        sim_->setName(visible ? sim_label_ : "");
        for (auto* m : legend_->markers(sim_)) m->setVisible(visible);
    }

    if (ref_) {
        const bool visible = show_ref_ && ref_live_ && !ref_label_.isEmpty();
        ref_->setName(visible ? ref_label_ : "");
        for (auto* m : legend_->markers(ref_)) m->setVisible(visible);
    }
}

void ChartBase::applyTheme(const prop_arm::ui::style::ThemeSpec& spec) {
    theme_ = spec;

    theme_applier_.apply(*chart_,
                         legend_,
                         *x_,
                         *y_,
                         real_,
                         sim_,
                         ref_,
                         spec,
                         show_sim_,
                         show_ref_,
                         real_live_,
                         sim_live_,
                         ref_live_,
                         integer_y_axis_);

    if (sim_) sim_->setVisible(show_sim_);
    if (ref_) ref_->setVisible(show_ref_);
}

void ChartBase::setTitle(const QString& title) {
    title_ = title;
    if (chart_) chart_->setTitle(title_);
}

void ChartBase::setYLabel(const QString& y_label) {
    y_label_ = y_label;
    if (y_) y_->setTitleText(y_label_);
}

void ChartBase::setXLabel(const QString& x_label) {
    x_label_ = x_label;
    if (x_) x_->setTitleText(x_label_);
}

void ChartBase::setShowSim(bool enabled) {
    const bool was = show_sim_;
    show_sim_ = enabled;
    if (sim_) sim_->setVisible(show_sim_);
    updateLegend_();
    needs_update_ = true;
    if (show_sim_ && !was) onUpdateTimer_();
}

void ChartBase::setShowRef(bool enabled) {
    const bool was = show_ref_;
    show_ref_ = enabled;
    if (ref_) ref_->setVisible(show_ref_);
    updateLegend_();
    needs_update_ = true;
    if (show_ref_ && !was) onUpdateTimer_();
}

void ChartBase::setWindowSeconds(double seconds) {
    window_s_ = std::max(0.5, seconds);
    window_.setLimits({window_s_, max_points_});
    needs_update_ = true;
}

void ChartBase::setMaxPoints(int max_points) {
    auto_points_enabled_ = false;
    max_points_ = std::max(kMinPoints, max_points);
    window_.setLimits({window_s_, max_points_});
    needs_update_ = true;
}

void ChartBase::setAutoRange(bool enabled) {
    auto_range_ = enabled;
    if (enabled) {
        y_range_locked_ = false;
    }
    needs_update_ = true;
}

void ChartBase::setSmoothCurves(bool enabled) {
    if (smooth_curves_ == enabled) return;
    smooth_curves_ = enabled;

    curve_.setMode(smooth_curves_
                   ? prop_arm::core::charts::CurveBuilder::Mode::MonotoneCubic
                   : prop_arm::core::charts::CurveBuilder::Mode::Raw);

    rebuildSeries_();
    applyTheme(theme_);
    needs_update_ = true;
}

void ChartBase::clear() {
    window_.clear();

    if (real_) real_->clear();
    if (sim_)  sim_->clear();
    if (ref_)  ref_->clear();

    real_series_buffer_.clear();
    sim_series_buffer_.clear();
    ref_series_buffer_.clear();
    real_spline_buffer_.clear();
    sim_spline_buffer_.clear();
    ref_spline_buffer_.clear();

    value_filter_.reset();
    rate_estimator_.clear();

    hover_active_ = false;
    needs_update_ = false;

    x_->setRange(0.0, window_s_);
    if (y_range_locked_) y_->setRange(y_min_locked_, y_max_locked_);
    else if (auto_range_) y_->setRange(-1.0, 1.0);

    cleanupHoverElements_();
    updateLegend_();
}

void ChartBase::append(double t_sec, double y_val, prop_arm::models::Domain domain) {
    if (!std::isfinite(t_sec) || !std::isfinite(y_val)) return;

    if (domain == prop_arm::models::Domain::Ref) {
        rate_estimator_.observe(domain, t_sec);
        window_.append(t_sec, y_val, prop_arm::models::Domain::Ref);
        ref_live_ = true;
        needs_update_ = true;
        return;
    }

    const auto prev_dt = rate_estimator_.dtEma(domain);
    rate_estimator_.observe(domain, t_sec);

    const double dt = prev_dt.value_or(0.0);
    y_val = value_filter_.filter(domain, y_val, dt);

    window_.append(t_sec, y_val, domain);
    needs_update_ = true;
}

void ChartBase::appendRef(double t_sec, double y_val) {
    if (!std::isfinite(t_sec) || !std::isfinite(y_val)) return;

    rate_estimator_.observe(prop_arm::models::Domain::Ref, t_sec);
    window_.append(t_sec, y_val, prop_arm::models::Domain::Ref);

    ref_live_ = true;
    needs_update_ = true;
}

void ChartBase::onUpdateTimer_() {
    if (!needs_update_) return;

    if (auto_points_enabled_) {
        auto_points_counter_++;
        if (auto_points_counter_ >= kAutoPointsUpdatePeriod) {
            auto_points_counter_ = 0;

            std::optional<double> dt;
            if (real_live_) dt = rate_estimator_.dtEma(prop_arm::models::Domain::Real);
            if (!dt.has_value() && show_sim_ && sim_live_)
                dt = rate_estimator_.dtEma(prop_arm::models::Domain::Sim);
            if (!dt.has_value() && show_ref_ && ref_live_)
                dt = rate_estimator_.dtEma(prop_arm::models::Domain::Ref);

            if (dt.has_value() && dt.value() > 0.0) {
                int new_max = prop_arm::core::charts::SampleRateEstimator::recommendedMaxPoints(window_s_, dt.value());
                new_max = clampi(new_max, kMinPoints, kMaxPointsCap);

                if (std::abs(new_max - max_points_) > std::max(200, max_points_ / 5)) {
                    max_points_ = new_max;
                    window_.setLimits({window_s_, max_points_});
                }
            }
        }
    }

    updateSeriesOptimized_();
    updateAxes_();

    if (hover_active_) updateHoverDisplay_();

    needs_update_ = false;
    emit dataUpdated();
}

void ChartBase::updateSeriesOptimized_() {
    if (!real_ || !sim_ || !ref_) return;

    const auto& real_buf = window_.real();
    const auto& sim_buf  = window_.sim();
    const auto& ref_buf  = window_.ref();

    if (real_live_ && !real_buf.empty()) {
        if (smooth_curves_) {
            curve_.build(real_buf, real_spline_buffer_);
            real_->replace(real_spline_buffer_);
        } else {
            fastClear(real_series_buffer_);
            real_series_buffer_.reserve(static_cast<int>(real_buf.size()));
            for (const auto& p : real_buf) real_series_buffer_.append(p);
            real_->replace(real_series_buffer_);
        }
    } else {
        real_->clear();
    }

    if (show_sim_ && sim_live_ && !sim_buf.empty()) {
        if (smooth_curves_) {
            curve_.build(sim_buf, sim_spline_buffer_);
            sim_->replace(sim_spline_buffer_);
        } else {
            fastClear(sim_series_buffer_);
            sim_series_buffer_.reserve(static_cast<int>(sim_buf.size()));
            for (const auto& p : sim_buf) sim_series_buffer_.append(p);
            sim_->replace(sim_series_buffer_);
        }
    } else {
        sim_->clear();
    }

    if (show_ref_ && ref_live_ && !ref_buf.empty()) {
        if (smooth_curves_) {
            curve_.build(ref_buf, ref_spline_buffer_);
            ref_->replace(ref_spline_buffer_);
        } else {
            fastClear(ref_series_buffer_);
            ref_series_buffer_.reserve(static_cast<int>(ref_buf.size()));
            for (const auto& p : ref_buf) ref_series_buffer_.append(p);
            ref_->replace(ref_series_buffer_);
        }
    } else {
        ref_->clear();
    }

    updateLegend_();
}

void ChartBase::updateAxes_() {
    const auto span = window_.span();
    const double t_max = span.t_max;

    if (t_max <= 0.0) x_->setRange(0.0, window_s_);
    else if (t_max < window_s_) x_->setRange(0.0, window_s_);
    else x_->setRange(t_max - window_s_, t_max);

    if (y_range_locked_) {
        y_->setRange(y_min_locked_, y_max_locked_);
        return;
    }
    if (!auto_range_) return;

    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    auto scan = [&](const std::deque<QPointF>& buf) {
        for (const auto& p : buf) {
            y_min = std::min(y_min, p.y());
            y_max = std::max(y_max, p.y());
        }
    };

    if (real_live_) scan(window_.real());
    if (show_sim_ && sim_live_) scan(window_.sim());
    if (show_ref_ && ref_live_) scan(window_.ref());

    if (!std::isfinite(y_min) || !std::isfinite(y_max) || std::abs(y_max - y_min) < kTiny) {
        y_min = -1.0;
        y_max =  1.0;
    } else {
        const double pad = 0.08 * (y_max - y_min);
        y_min -= pad;
        y_max += pad;
    }

    if (y_bounds_enabled_) {
        y_min = std::clamp(y_min, y_min_bound_, y_max_bound_);
        y_max = std::clamp(y_max, y_min_bound_, y_max_bound_);
        if (y_min >= y_max) {
            y_min = y_min_bound_;
            y_max = y_max_bound_;
        }
    }

    y_->setRange(y_min, y_max);
}

void ChartBase::onHoverUpdate(QPointF /*chart_pos*/, QPoint viewport_pos, bool valid) {
    hover_active_ = valid;
    hover_viewport_pos_ = viewport_pos;

    if (valid) updateHoverDisplay_();
    else {
        cleanupHoverElements_();
        emit hoveredTextChanged("");
    }
}

void ChartBase::updateHoverDisplay_() {
    if (!view_ || !view_->scene() || !chart_ || !hover_active_) return;

    const QPointF scene_pos = view_->mapToScene(hover_viewport_pos_);
    const QPointF chart_item_pos = chart_->mapFromScene(scene_pos);
    const QPointF hover_value = chart_->mapToValue(chart_item_pos);

    const double t_abs = hover_value.x();

    const auto r = prop_arm::core::charts::HoverInterpolator::yAt(window_.real(), t_abs);
    const auto s = show_sim_ ? prop_arm::core::charts::HoverInterpolator::yAt(window_.sim(), t_abs) : std::nullopt;
    const auto f = show_ref_ ? prop_arm::core::charts::HoverInterpolator::yAt(window_.ref(), t_abs) : std::nullopt;

    const bool show_real = r.has_value() && real_live_ && !real_label_.isEmpty();
    const bool show_sim_val = s.has_value() && show_sim_ && sim_live_ && !sim_label_.isEmpty();
    const bool show_ref_val = f.has_value() && show_ref_ && ref_live_ && !ref_label_.isEmpty();

    if (!show_real && !show_sim_val && !show_ref_val) {
        cleanupHoverElements_();
        return;
    }

    const QPointF p_real = r.value_or(QPointF(t_abs, 0.0));
    const QPointF p_sim  = s.value_or(QPointF(t_abs, 0.0));
    const QPointF p_ref  = f.value_or(QPointF(t_abs, 0.0));

    const QRectF plot_area = chart_->plotArea();
    const QRectF plot_scene(chart_->mapToScene(plot_area.topLeft()),
                            chart_->mapToScene(plot_area.bottomRight()));

    QGraphicsScene* scene = view_->scene();

    // Vertical line
    if (!view_->hover_line_) {
        QColor line_color = theme_.text_muted;
        line_color.setAlpha(200);
        QPen pen(line_color, 2, Qt::DashLine);
        pen.setCapStyle(Qt::RoundCap);
        view_->hover_line_ = scene->addLine(0, 0, 0, 0, pen);
    }

    const QPointF x_scene_ref = chart_->mapToScene(chart_->mapToPosition(QPointF(t_abs, y_->min())));
    view_->hover_line_->setLine(x_scene_ref.x(), plot_scene.top(),
                                x_scene_ref.x(), plot_scene.bottom());

    // Real point
    if (show_real) {
        if (!view_->hover_point_) {
            view_->hover_point_ = scene->addEllipse(0, 0, 0, 0,
                                                    QPen(theme_.accent, 2),
                                                    QBrush(theme_.accent));
        }
        const QPointF ps = chart_->mapToScene(chart_->mapToPosition(p_real));
        view_->hover_point_->setVisible(true);
        view_->hover_point_->setRect(ps.x() - 5, ps.y() - 5, 10, 10);
    } else if (view_->hover_point_) {
        view_->hover_point_->setVisible(false);
    }

    // Sim point
    if (show_sim_val) {
        if (!view_->hover_point_sim_) {
            QPen ps(theme_.accent2, 2);
            view_->hover_point_sim_ = scene->addEllipse(0, 0, 0, 0, ps, QBrush(theme_.accent2));
        }
        const QPointF ps = chart_->mapToScene(chart_->mapToPosition(p_sim));
        view_->hover_point_sim_->setVisible(true);
        view_->hover_point_sim_->setRect(ps.x() - 5, ps.y() - 5, 10, 10);
    } else if (view_->hover_point_sim_) {
        view_->hover_point_sim_->setVisible(false);
    }

    // Ref point
    if (show_ref_val) {
        if (!view_->hover_point_ref_) {
            QPen pr(theme_.ref_color, 2);
            view_->hover_point_ref_ = scene->addEllipse(0, 0, 0, 0, pr, QBrush(theme_.ref_color));
        } else {
            view_->hover_point_ref_->setPen(QPen(theme_.ref_color, 2));
            view_->hover_point_ref_->setBrush(QBrush(theme_.ref_color));
        }
        const QPointF ps = chart_->mapToScene(chart_->mapToPosition(p_ref));
        view_->hover_point_ref_->setVisible(true);
        view_->hover_point_ref_->setRect(ps.x() - 5, ps.y() - 5, 10, 10);
    } else if (view_->hover_point_ref_) {
        view_->hover_point_ref_->setVisible(false);
    }

    QString text;
    text += QString("Time: %1 s").arg(t_abs, 0, 'f', 2);

    auto fmtY = [this](double v) -> QString {
        if (integer_y_axis_) return QString::number(static_cast<int>(std::llround(v)));
        return QString::number(v, 'f', 3);
    };

    if (show_real) text += QString("\n%1: %2").arg(real_label_, fmtY(p_real.y()));
    if (show_sim_val) text += QString("\n%1: %2").arg(sim_label_, fmtY(p_sim.y()));
    if (show_ref_val) text += QString("\n%1: %2").arg(ref_label_, fmtY(p_ref.y()));

    if (!view_->hover_text_) {
        view_->hover_text_ = scene->addText("");
        view_->hover_text_->setDefaultTextColor(theme_.text);
        QFont font = view_->hover_text_->font();
        font.setPointSize(10);
        font.setBold(false);
        view_->hover_text_->setFont(font);
    }

    view_->hover_text_->setPlainText(text);

    const QRectF text_rect = view_->hover_text_->boundingRect();
    QPointF anchor_scene = chart_->mapToScene(chart_->mapToPosition(
                               show_real ? p_real : (show_ref_val ? p_ref : p_sim)));

    QPointF text_pos(anchor_scene.x() + 10, anchor_scene.y() - text_rect.height() - 5);

    if (text_pos.x() + text_rect.width() > plot_scene.right())
        text_pos.setX(anchor_scene.x() - text_rect.width() - 10);
    if (text_pos.y() < plot_scene.top())
        text_pos.setY(anchor_scene.y() + 10);

    view_->hover_text_->setPos(text_pos);

    emit hoveredTextChanged(text);
}

void ChartBase::cleanupHoverElements_() {
    if (view_) view_->clearHoverElements();
}

void ChartBase::setYRange(double y_min, double y_max) {
    if (y_min > y_max) std::swap(y_min, y_max);
    y_range_locked_ = true;
    y_min_locked_ = y_min;
    y_max_locked_ = y_max;
    auto_range_ = false;
    y_->setRange(y_min, y_max);
}

void ChartBase::clearYRange() {
    y_range_locked_ = false;
    auto_range_ = true;
    needs_update_ = true;
}

void ChartBase::setYBounds(double y_min, double y_max) {
    if (y_min > y_max) std::swap(y_min, y_max);
    y_bounds_enabled_ = true;
    y_min_bound_ = y_min;
    y_max_bound_ = y_max;
    y_range_locked_ = false;
    auto_range_ = true;
    needs_update_ = true;
}

void ChartBase::clearYBounds() {
    y_bounds_enabled_ = false;
    needs_update_ = true;
}

void ChartBase::setRealLive(bool live) {
    if (real_live_ == live) return;
    real_live_ = live;
    updateLegend_();
    needs_update_ = true;
}

void ChartBase::setSimLive(bool live) {
    if (sim_live_ == live) return;
    sim_live_ = live;
    updateLegend_();
    needs_update_ = true;
}

void ChartBase::setRefLive(bool live) {
    if (ref_live_ == live) return;
    ref_live_ = live;
    updateLegend_();
    needs_update_ = true;
}

void ChartBase::setIntegerYAxis(bool enabled) {
    if (integer_y_axis_ == enabled) return;
    integer_y_axis_ = enabled;
    applyTheme(theme_);
    needs_update_ = true;
    onUpdateTimer_();
}

} // namespace prop_arm::ui::widgets
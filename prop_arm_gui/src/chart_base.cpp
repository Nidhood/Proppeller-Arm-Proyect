#include "prop_arm_gui/chart_base.hpp"
#include <QMetaObject>
#include <QMutexLocker>
#include <QDateTime>
#include <QGraphicsScene>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QPen>
#include <algorithm>
#include <cmath>

// Define static color constants
const QColor ChartBase::BACKGROUND_DARK = QColor(0, 0, 0);
const QColor ChartBase::PLOT_BACKGROUND = QColor(20, 20, 20);
const QColor ChartBase::GRID_PRIMARY = QColor(60, 60, 60);
const QColor ChartBase::GRID_SECONDARY = QColor(40, 40, 40);
const QColor ChartBase::GRID_MINOR = QColor(30, 30, 30);
const QColor ChartBase::TEXT_PRIMARY = QColor(255, 255, 255);
const QColor ChartBase::TEXT_SECONDARY = QColor(200, 200, 200);
const QColor ChartBase::ACCENT_YELLOW = QColor(255, 255, 0);
const QColor ChartBase::ACCENT_ORANGE = QColor(255, 165, 0);
const QColor ChartBase::ACCENT_GREEN = QColor(0, 255, 0);
const QColor ChartBase::ACCENT_BLUE = QColor(0, 150, 255);
const QColor ChartBase::ACCENT_PURPLE = QColor(160, 0, 255);
const QColor ChartBase::ACCENT_RED = QColor(255, 0, 0);

// HoverChartView implementation
HoverChartView::HoverChartView(QWidget *parent)
    : QChartView(parent),
      hover_enabled_(true),
      mouse_in_plot_area_(false),
      hover_line_(nullptr),
      hover_point_(nullptr),
      hover_text_(nullptr)
{
    setMouseTracking(true);
    setRenderHint(QPainter::Antialiasing);
}

HoverChartView::~HoverChartView()
{
    clearHoverElements();
}

void HoverChartView::mouseMoveEvent(QMouseEvent *event)
{
    if (!hover_enabled_)
    {
        QChartView::mouseMoveEvent(event);
        return;
    }

    QPoint pos = event->pos();
    bool in_plot = isInPlotArea(pos);

    if (in_plot)
    {
        hover_position_scene_ = pos;
        if (chart())
        {
            hover_position_chart_ = chart()->mapToValue(pos);
        }
        mouse_in_plot_area_ = true;
        emit hoverUpdate(hover_position_chart_, hover_position_scene_, true);
    }
    else
    {
        mouse_in_plot_area_ = false;
        emit hoverUpdate(QPointF(), QPoint(), false);
    }

    QChartView::mouseMoveEvent(event);
}

void HoverChartView::leaveEvent(QEvent *event)
{
    mouse_in_plot_area_ = false;
    emit hoverUpdate(QPointF(), QPoint(), false);
    QChartView::leaveEvent(event);
}

bool HoverChartView::isInPlotArea(const QPoint &pos) const
{
    if (!chart())
        return false;

    QRectF plot_area = chart()->plotArea();
    return plot_area.contains(pos);
}

void HoverChartView::clearHoverElements()
{
    if (scene())
    {
        if (hover_line_)
        {
            scene()->removeItem(hover_line_);
            delete hover_line_;
            hover_line_ = nullptr;
        }
        if (hover_point_)
        {
            scene()->removeItem(hover_point_);
            delete hover_point_;
            hover_point_ = nullptr;
        }
        if (hover_text_)
        {
            scene()->removeItem(hover_text_);
            delete hover_text_;
            hover_text_ = nullptr;
        }
    }
}

// ChartBase implementation
ChartBase::ChartBase(const ChartConfig &config, QWidget *parent)
    : QWidget(parent),
      config_(config),
      start_time_(QDateTime::currentMSecsSinceEpoch() / 1000.0),
      chart_(nullptr),
      chart_view_(nullptr),
      main_series_(nullptr),
      trend_series_(nullptr),
      smooth_series_(nullptr),
      x_axis_(nullptr),
      y_axis_(nullptr),
      data_mutex_(new QMutex()),
      pool_write_index_(0),
      optimal_buffer_size_(1000),
      update_timer_(nullptr),
      last_update_time_(0),
      last_hover_update_time_(0),
      update_pending_(false),
      hover_update_pending_(false),
      data_initialized_(false),
      hover_active_(false),
      hover_persistent_(false),
      hover_fixed_time_absolute_(-1.0), // FIXED: Usar tiempo absoluto
      cached_hover_value_(0.0),
      cached_hover_time_(-1.0)
{
    initializeMemoryPool();
    createLayout();
    setupChart();
    setupAxes();
    setupSeries();
    setupAdvancedGrid();
    applyProfessionalTheme();
    initializeWithZeroData();

    // Setup update timer
    update_timer_ = new QTimer(this);
    update_timer_->setSingleShot(false);
    update_timer_->setInterval(50); // 20 FPS
    connect(update_timer_, &QTimer::timeout, this, &ChartBase::performUpdate);
    update_timer_->start();
}

ChartBase::~ChartBase()
{
    if (update_timer_)
    {
        update_timer_->stop();
    }

    for (auto *line : minor_grid_lines_x_)
    {
        delete line;
    }
    for (auto *line : minor_grid_lines_y_)
    {
        delete line;
    }

    delete data_mutex_;
}

void ChartBase::createLayout()
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    chart_view_ = new HoverChartView(this);
    layout->addWidget(chart_view_);

    connect(chart_view_, &HoverChartView::hoverUpdate,
            this, &ChartBase::onHoverUpdate);
}

void ChartBase::setupChart()
{
    chart_ = new QChart();
    chart_->setTitle(config_.title);
    chart_->setAnimationOptions(QChart::NoAnimation);
    chart_view_->setChart(chart_);
}

void ChartBase::setupAxes()
{
    x_axis_ = new QValueAxis();
    x_axis_->setTitleText("Time (s)");
    x_axis_->setRange(0, config_.time_window_sec);
    x_axis_->setGridLineVisible(config_.show_grid);
    x_axis_->setMinorGridLineVisible(config_.show_minor_grid);
    x_axis_->setTickCount(6);
    x_axis_->setMinorTickCount(5);

    y_axis_ = new QValueAxis();
    y_axis_->setTitleText(config_.y_label + " (" + config_.units + ")");
    y_axis_->setRange(config_.y_min, config_.y_max);
    y_axis_->setGridLineVisible(config_.show_grid);
    y_axis_->setMinorGridLineVisible(config_.show_minor_grid);
    y_axis_->setTickCount(6);
    y_axis_->setMinorTickCount(5);

    chart_->addAxis(x_axis_, Qt::AlignBottom);
    chart_->addAxis(y_axis_, Qt::AlignLeft);
}

void ChartBase::setupSeries()
{
    if (config_.use_smooth_curves)
    {
        smooth_series_ = new QSplineSeries();
        smooth_series_->setName("Data");
        smooth_series_->setColor(config_.primary_color);

        QPen pen(config_.primary_color);
        pen.setWidth(2);
        smooth_series_->setPen(pen);

        chart_->addSeries(smooth_series_);
        smooth_series_->attachAxis(x_axis_);
        smooth_series_->attachAxis(y_axis_);
    }
    else
    {
        main_series_ = new QLineSeries();
        main_series_->setName("Data");
        main_series_->setColor(config_.primary_color);

        QPen pen(config_.primary_color);
        pen.setWidth(1);
        main_series_->setPen(pen);

        chart_->addSeries(main_series_);
        main_series_->attachAxis(x_axis_);
        main_series_->attachAxis(y_axis_);
    }

    series_points_buffer_.reserve(config_.max_points);
    spline_points_buffer_.reserve(config_.max_points);
}

void ChartBase::setupAdvancedGrid()
{
    if (!config_.show_minor_grid)
        return;
}

void ChartBase::applyProfessionalTheme()
{
    chart_->setBackgroundBrush(QBrush(BACKGROUND_DARK));
    chart_->setPlotAreaBackgroundBrush(QBrush(PLOT_BACKGROUND));
    chart_->setPlotAreaBackgroundVisible(true);

    x_axis_->setGridLineColor(GRID_PRIMARY);
    x_axis_->setMinorGridLineColor(GRID_MINOR);
    x_axis_->setLabelsColor(TEXT_PRIMARY);
    x_axis_->setTitleBrush(QBrush(TEXT_PRIMARY));

    y_axis_->setGridLineColor(GRID_PRIMARY);
    y_axis_->setMinorGridLineColor(GRID_MINOR);
    y_axis_->setLabelsColor(TEXT_PRIMARY);
    y_axis_->setTitleBrush(QBrush(TEXT_PRIMARY));

    chart_->setTitleBrush(QBrush(TEXT_PRIMARY));
    QFont titleFont = chart_->titleFont();
    titleFont.setPointSize(12);
    titleFont.setBold(true);
    chart_->setTitleFont(titleFont);
}

void ChartBase::updateAxes()
{
    if (!x_axis_ || !y_axis_)
        return;

    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    double time_range = current_time - start_time_;

    if (time_range > config_.time_window_sec)
    {
        x_axis_->setRange(time_range - config_.time_window_sec, time_range);
    }
    else
    {
        x_axis_->setRange(0, config_.time_window_sec);
    }

    if (config_.auto_scale && !data_points_.empty())
    {
        auto [min_it, max_it] = std::minmax_element(
            data_points_.begin(), data_points_.end(),
            [](const DataPoint &a, const DataPoint &b)
            {
                return a.value < b.value;
            });

        double margin = (max_it->value - min_it->value) * 0.1;
        y_axis_->setRange(min_it->value - margin, max_it->value + margin);
    }
}

void ChartBase::initializeMemoryPool()
{
    optimal_buffer_size_ = std::min(static_cast<size_t>(2000), config_.max_points * 2);
    series_points_buffer_.clear();
    series_points_buffer_.reserve(optimal_buffer_size_);
    spline_points_buffer_.clear();
    spline_points_buffer_.reserve(optimal_buffer_size_);
    pool_write_index_ = 0;
}

void ChartBase::initializeWithOptimizedData()
{
    series_points_buffer_.clear();
    if (!data_points_.empty())
    {
        for (const auto &point : data_points_)
        {
            double relative_time = point.timestamp - start_time_;
            series_points_buffer_.append(QPointF(relative_time, point.value));
        }
    }
}

void ChartBase::initializeWithZeroData()
{
    if (data_initialized_)
        return;

    QMutexLocker locker(data_mutex_);

    data_points_.clear();

    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    start_time_ = current_time;

    int num_initial_points = static_cast<int>(config_.time_window_sec * 10);
    for (int i = 0; i < num_initial_points; ++i)
    {
        double timestamp = start_time_ + (i * config_.time_window_sec) / num_initial_points;
        double initial_value = (config_.y_min + config_.y_max) / 2.0;
        data_points_.push_back(DataPoint(timestamp, initial_value));
    }

    data_initialized_ = true;
    update_pending_ = true;
}

void ChartBase::addDataPoint(double value, double timestamp)
{
    QMutexLocker locker(data_mutex_);

    if (timestamp < 0)
    {
        timestamp = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    }

    DataPoint point(timestamp, value);

    if (!data_initialized_)
    {
        initializeWithZeroData();
    }

    data_points_.push_back(point);

    double current_time = timestamp;
    double cutoff_time = current_time - config_.time_window_sec;

    while (!data_points_.empty() && data_points_.front().timestamp < cutoff_time)
    {
        data_points_.pop_front();
    }

    while (data_points_.size() > config_.max_points)
    {
        data_points_.pop_front();
    }

    update_pending_ = true;
}

void ChartBase::updateSeriesOptimized()
{
    series_points_buffer_.clear();

    for (const auto &point : data_points_)
    {
        double relative_time = point.timestamp - start_time_;
        series_points_buffer_.append(QPointF(relative_time, point.value));
    }

    if (config_.use_smooth_curves && smooth_series_)
    {
        updateSplineSeries();
    }
    else if (main_series_)
    {
        main_series_->replace(series_points_buffer_);
    }
}

void ChartBase::updateSplineSeries()
{
    if (!smooth_series_ || series_points_buffer_.isEmpty())
        return;

    if (series_points_buffer_.size() <= 10)
    {
        smooth_series_->replace(series_points_buffer_);
        return;
    }

    std::vector<QPointF> control_points;
    control_points.reserve(series_points_buffer_.size());

    for (const auto &point : series_points_buffer_)
    {
        control_points.push_back(point);
    }

    auto spline_points = calculateSplinePoints(control_points);

    spline_points_buffer_.clear();
    for (const auto &point : spline_points)
    {
        spline_points_buffer_.append(point);
    }

    smooth_series_->replace(spline_points_buffer_);
}

std::vector<QPointF> ChartBase::calculateSplinePoints(const std::vector<QPointF> &control_points) const
{
    if (control_points.size() < 3)
        return control_points;

    std::vector<QPointF> result;
    result.reserve(control_points.size() * 3);

    for (size_t i = 0; i < control_points.size() - 1; ++i)
    {
        const QPointF &p0 = (i > 0) ? control_points[i - 1] : control_points[i];
        const QPointF &p1 = control_points[i];
        const QPointF &p2 = control_points[i + 1];
        const QPointF &p3 = (i < control_points.size() - 2) ? control_points[i + 2] : control_points[i + 1];

        const int segments = 3;
        for (int j = 0; j < segments; ++j)
        {
            double t = static_cast<double>(j) / segments;
            double t2 = t * t;
            double t3 = t2 * t;

            double b0 = -0.5 * t3 + t2 - 0.5 * t;
            double b1 = 1.5 * t3 - 2.5 * t2 + 1.0;
            double b2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
            double b3 = 0.5 * t3 - 0.5 * t2;

            double x = b0 * p0.x() + b1 * p1.x() + b2 * p2.x() + b3 * p3.x();
            double y = b0 * p0.y() + b1 * p1.y() + b2 * p2.y() + b3 * p3.y();

            result.emplace_back(x, y);
        }
    }

    result.push_back(control_points.back());
    return result;
}

void ChartBase::performUpdate()
{
    // Update series/axes only when data changed
    if (update_pending_)
    {
        QMutexLocker locker(data_mutex_);
        updateSeriesOptimized();
        updateAxes();
        update_pending_ = false;
        emit dataUpdated();
    }

    // Always refresh persistent hover so the vertical line/label
    // track the sliding window even if the mouse is still.
    if (hover_persistent_ && hover_fixed_time_absolute_ > 0.0)
    {
        updatePersistentHover();
    }
}

// FIXED: Guardar tiempo absoluto, no relativo
void ChartBase::onHoverUpdate(QPointF chart_position, QPoint scene_position, bool valid)
{
    current_chart_pos_ = chart_position;
    current_scene_pos_ = scene_position;
    hover_active_ = valid;

    if (valid)
    {
        // CRITICAL FIX: Guardar el tiempo ABSOLUTO, no el relativo del chart
        hover_fixed_time_absolute_ = start_time_ + chart_position.x();
        hover_persistent_ = true;
        hover_fixed_scene_x_ = scene_position.x();
        updateHoverDisplay();
    }
    else
    {
        hover_active_ = false;
        hover_persistent_ = false;
        hover_fixed_time_absolute_ = -1.0;
        cleanupHoverElements();
    }
}

// FIXED: Usar tiempo absoluto fijo para calcular posición relativa actual
void ChartBase::updateHoverDisplay()
{
    if (!chart_view_ || !chart_view_->scene())
        return;

    if (hover_fixed_time_absolute_ <= 0.0)
        return;

    // Calcular tiempo relativo actual para la posición fija absoluta
    QPointF chart_at_x = sceneToChartCoords(QPoint(hover_fixed_scene_x_, current_scene_pos_.y()));
    double current_relative_time = chart_at_x.x();
    // Keep absolute time consistent with current mapping
    hover_fixed_time_absolute_ = start_time_ + current_relative_time;
    // Verificar que esté dentro del rango visible actual
    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    double time_range = current_time - start_time_;

    double x_min, x_max;
    if (time_range > config_.time_window_sec)
    {
        x_min = time_range - config_.time_window_sec;
        x_max = time_range;
    }
    else
    {
        x_min = 0;
        x_max = config_.time_window_sec;
    }

    // Solo mostrar hover si la posición fija está en el rango visible
    if (current_relative_time < x_min || current_relative_time > x_max)
    {
        cleanupHoverElements();
        return;
    }

    // Encontrar valor interpolado para el tiempo absoluto fijo
    QPointF snapped_point = findNearestPointOnCurve(hover_fixed_time_absolute_);
    snapped_chart_pos_ = QPointF(current_relative_time, snapped_point.y());

    // Convertir a coordenadas de escena
    QPoint snapped_scene_pos = chartToSceneCoords(snapped_chart_pos_);

    // Crear elementos con el tiempo original para mostrar
    double display_time = current_relative_time;
    createHoverElements(chart_view_->scene(), snapped_chart_pos_,
                        snapped_scene_pos, snapped_point.y(), display_time);
}

void ChartBase::updatePersistentHover()
{
    if (!hover_persistent_ || hover_fixed_time_absolute_ <= 0.0)
        return;

    updateHoverDisplay();
}

void ChartBase::cleanupHoverElements()
{
    if (chart_view_)
    {
        chart_view_->clearHoverElements();
    }
}

void ChartBase::createHoverElements(QGraphicsScene *scene, const QPointF &,
                                    const QPoint &scene_pos, double value, double time)
{
    if (!scene || !chart_view_)
        return;

    cleanupHoverElements();

    if (!chart_->plotArea().contains(scene_pos))
        return;

    // Línea vertical
    chart_view_->hover_line_ = scene->addLine(
        scene_pos.x(), chart_->plotArea().top(),
        scene_pos.x(), chart_->plotArea().bottom(),
        QPen(TEXT_SECONDARY, 1, Qt::DashLine));

    // Punto en la curva
    chart_view_->hover_point_ = scene->addEllipse(
        scene_pos.x() - 4, scene_pos.y() - 4, 8, 8,
        QPen(config_.primary_color, 2),
        QBrush(config_.primary_color));

    // Texto con información
    QString text = QString("Time: %1s\nValue: %2 %3")
                       .arg(time, 0, 'f', 2)
                       .arg(value, 0, 'f', 3)
                       .arg(config_.units);

    chart_view_->hover_text_ = scene->addText(text);
    chart_view_->hover_text_->setDefaultTextColor(TEXT_PRIMARY);

    QFont font = chart_view_->hover_text_->font();
    font.setPointSize(10);
    font.setBold(true);
    chart_view_->hover_text_->setFont(font);

    // Posicionamiento inteligente del texto
    QRectF text_rect = chart_view_->hover_text_->boundingRect();
    QPointF text_pos(scene_pos.x() + 15, scene_pos.y() - text_rect.height() - 5);

    if (text_pos.x() + text_rect.width() > chart_->plotArea().right())
    {
        text_pos.setX(scene_pos.x() - text_rect.width() - 15);
    }

    if (text_pos.y() < chart_->plotArea().top())
    {
        text_pos.setY(scene_pos.y() + 15);
    }

    if (text_pos.y() + text_rect.height() > chart_->plotArea().bottom())
    {
        text_pos.setY(chart_->plotArea().bottom() - text_rect.height() - 5);
    }

    chart_view_->hover_text_->setPos(text_pos);
}

// FIXED: Buscar por tiempo absoluto
QPointF ChartBase::findNearestPointOnCurve(double absolute_target_time) const
{
    if (data_points_.empty())
        return QPointF(0, (config_.y_min + config_.y_max) / 2.0);

    // Buscar por tiempo absoluto directamente
    auto it = std::lower_bound(data_points_.begin(), data_points_.end(), absolute_target_time,
                               [](const DataPoint &point, double time)
                               {
                                   return point.timestamp < time;
                               });

    if (it == data_points_.begin())
    {
        return QPointF(it->timestamp - start_time_, it->value);
    }
    if (it == data_points_.end())
    {
        return QPointF(data_points_.back().timestamp - start_time_, data_points_.back().value);
    }

    // Interpolación lineal
    auto prev_it = it - 1;
    double t1 = prev_it->timestamp;
    double t2 = it->timestamp;
    double v1 = prev_it->value;
    double v2 = it->value;

    double factor = (absolute_target_time - t1) / (t2 - t1);
    factor = std::max(0.0, std::min(1.0, factor));

    double interpolated_value = v1 + factor * (v2 - v1);
    return QPointF(absolute_target_time - start_time_, interpolated_value);
}

double ChartBase::findValueAtScenePosition(const QPoint &scene_pos)
{
    Q_UNUSED(scene_pos)
    return interpolateValue(current_chart_pos_.x());
}

double ChartBase::interpolateValue(double target_time) const
{
    if (data_points_.empty())
        return 0.0;

    auto it = std::lower_bound(data_points_.begin(), data_points_.end(), target_time,
                               [this](const DataPoint &point, double time)
                               {
                                   return (point.timestamp - start_time_) < time;
                               });

    if (it == data_points_.begin())
    {
        return it->value;
    }
    if (it == data_points_.end())
    {
        return data_points_.back().value;
    }

    auto prev_it = it - 1;
    double t1 = prev_it->timestamp - start_time_;
    double t2 = it->timestamp - start_time_;
    double v1 = prev_it->value;
    double v2 = it->value;

    double factor = (target_time - t1) / (t2 - t1);
    return v1 + factor * (v2 - v1);
}

QPointF ChartBase::sceneToChartCoords(const QPoint &scene_pos) const
{
    if (!chart_view_ || !chart_view_->chart())
        return QPointF();

    return chart_view_->chart()->mapToValue(scene_pos);
}

QPoint ChartBase::chartToSceneCoords(const QPointF &chart_pos) const
{
    if (!chart_view_ || !chart_view_->chart())
        return QPoint();

    QPointF scene_pos = chart_view_->chart()->mapToPosition(chart_pos);
    return QPoint(static_cast<int>(scene_pos.x()), static_cast<int>(scene_pos.y()));
}

void ChartBase::clearData()
{
    QMutexLocker locker(data_mutex_);
    data_points_.clear();
    start_time_ = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    last_update_time_ = 0;
    last_hover_update_time_ = 0;
    update_pending_ = false;
    hover_update_pending_ = false;
    data_initialized_ = false;

    // Reset hover state pero mantener posición si está activo
    if (hover_persistent_ && hover_fixed_time_absolute_ > 0.0)
    {
        cached_hover_value_ = 0.0;
        cached_hover_time_ = -1.0;
    }
    else
    {
        hover_active_ = false;
        hover_persistent_ = false;
        hover_fixed_time_absolute_ = -1.0;
        cached_hover_value_ = 0.0;
        cached_hover_time_ = -1.0;
    }

    pool_write_index_ = 0;

    initializeWithZeroData();
    initializeWithOptimizedData();

    QMetaObject::invokeMethod(this, [this]()
                              {
        if (main_series_) main_series_->clear();
        if (smooth_series_) smooth_series_->clear();
        x_axis_->setRange(0, config_.time_window_sec);
        y_axis_->setRange(config_.y_min, config_.y_max);
        
        if (!hover_persistent_) {
            cleanupHoverElements();
        }
        
        update_pending_ = true; }, Qt::QueuedConnection);
}

void ChartBase::setTimeWindow(double seconds)
{
    config_.time_window_sec = std::max(1.0, std::min(120.0, seconds));

    QMutexLocker locker(data_mutex_);
    data_points_.clear();
    data_initialized_ = false;
    initializeMemoryPool();
    initializeWithZeroData();
    initializeWithOptimizedData();
    update_pending_ = true;
}

void ChartBase::setShowMilliseconds(bool /*show_ms*/)
{
    config_.show_milliseconds = false;
    setupAxes();
    update_pending_ = true;
}

void ChartBase::setYRange(double min, double max)
{
    config_.y_min = min;
    config_.y_max = max;
    if (!config_.auto_scale)
    {
        QMetaObject::invokeMethod(this, [this, min, max]()
                                  { y_axis_->setRange(min, max); }, Qt::QueuedConnection);
    }
}

void ChartBase::setAutoScale(bool enabled)
{
    config_.auto_scale = enabled;
    if (!enabled)
    {
        QMetaObject::invokeMethod(this, [this]()
                                  { y_axis_->setRange(config_.y_min, config_.y_max); }, Qt::QueuedConnection);
    }
}

void ChartBase::setGridVisible(bool visible)
{
    config_.show_grid = visible;
    QMetaObject::invokeMethod(this, [this, visible]()
                              {
        x_axis_->setGridLineVisible(visible);
        y_axis_->setGridLineVisible(visible);
        x_axis_->setMinorGridLineVisible(visible && config_.show_minor_grid);
        y_axis_->setMinorGridLineVisible(visible && config_.show_minor_grid); }, Qt::QueuedConnection);
}

void ChartBase::setSmoothCurves(bool enabled)
{
    if (config_.use_smooth_curves == enabled)
        return;

    config_.use_smooth_curves = enabled;

    QMetaObject::invokeMethod(this, [this]()
                              {
        QVector<QPointF> current_data = series_points_buffer_;
        
        if (main_series_) {
            chart_->removeSeries(main_series_);
            delete main_series_;
            main_series_ = nullptr;
        }
        if (smooth_series_) {
            chart_->removeSeries(smooth_series_);
            delete smooth_series_;
            smooth_series_ = nullptr;
        }
        
        setupSeries();
        
        if (!current_data.isEmpty()) {
            if (config_.use_smooth_curves && smooth_series_) {
                smooth_series_->replace(current_data);
            } else if (main_series_) {
                main_series_->replace(current_data);
            }
        }
        
        update_pending_ = true; }, Qt::QueuedConnection);
}

void ChartBase::updateChart()
{
    update_pending_ = true;
}

void ChartBase::updateTheme()
{
    QMetaObject::invokeMethod(this, [this]()
                              { applyProfessionalTheme(); }, Qt::QueuedConnection);
}

#include "chart_base.moc"
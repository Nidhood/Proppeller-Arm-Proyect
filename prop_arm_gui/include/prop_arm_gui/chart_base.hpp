#pragma once

#include <QWidget>
#include <QChart>
#include <QChartView>
#include <QLineSeries>
#include <QSplineSeries>
#include <QValueAxis>
#include <QPainter>
#include <QColor>
#include <QFont>
#include <QFontDatabase>
#include <QDateTime>
#include <QTimer>
#include <QMutex>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <deque>
#include <memory>
#include <array>

struct DataPoint
{
    double timestamp;
    double value;
    QDateTime datetime;

    DataPoint() = default;
    DataPoint(double t, double v) : timestamp(t), value(v), datetime(QDateTime::currentDateTime()) {}
};

class HoverChartView : public QChartView
{
    Q_OBJECT

public:
    explicit HoverChartView(QWidget *parent = nullptr);
    ~HoverChartView();

    QPointF hover_position_chart_;
    QPoint hover_position_scene_;
    bool hover_enabled_;
    bool mouse_in_plot_area_;

    void clearHoverElements();

protected:
    void mouseMoveEvent(QMouseEvent *event) override;
    void leaveEvent(QEvent *event) override;
    bool isInPlotArea(const QPoint &pos) const;

signals:
    void hoverUpdate(QPointF chart_position, QPoint scene_position, bool valid);

public:
    QGraphicsLineItem *hover_line_;
    QGraphicsEllipseItem *hover_point_;
    QGraphicsTextItem *hover_text_;
};

class ChartBase : public QWidget
{
    Q_OBJECT

public:
    struct ChartConfig
    {
        QString title;
        QString y_label;
        QString units;
        QColor primary_color;
        QColor secondary_color;
        QColor sim_color; // Color para datos de simulación
        double y_min;
        double y_max;
        bool auto_scale;
        bool show_grid;
        double time_window_sec;
        size_t max_points;
        bool show_milliseconds;
        bool use_smooth_curves = true;
        bool show_minor_grid = true;
    };

    explicit ChartBase(const ChartConfig &config, QWidget *parent = nullptr);
    virtual ~ChartBase();

    // Data management - ahora con soporte para datos de simulación
    void addDataPoint(double value, double timestamp = -1);
    void addSimDataPoint(double value, double timestamp = -1); // NUEVO: datos de simulación
    void clearData();
    void setTimeWindow(double seconds);
    void setYRange(double min, double max);
    void setAutoScale(bool enabled);

    // Visual configuration
    void updateTheme();
    void setGridVisible(bool visible);
    void setShowMilliseconds(bool show_ms);
    void setSmoothCurves(bool enabled);

    // Updated neon color scheme
    static const QColor BACKGROUND_DARK;
    static const QColor PLOT_BACKGROUND;
    static const QColor GRID_PRIMARY;
    static const QColor GRID_SECONDARY;
    static const QColor GRID_MINOR;
    static const QColor TEXT_PRIMARY;
    static const QColor TEXT_SECONDARY;
    static const QColor ACCENT_YELLOW;
    static const QColor ACCENT_ORANGE;
    static const QColor ACCENT_GREEN;
    static const QColor ACCENT_BLUE;
    static const QColor ACCENT_PURPLE;
    static const QColor ACCENT_RED;

public slots:
    void updateChart();
    void performUpdate();
    void onHoverUpdate(QPointF chart_position, QPoint scene_position, bool valid);

signals:
    void dataUpdated();

protected:
    virtual void setupChart();
    virtual void applyProfessionalTheme();
    virtual void updateAxes();

    void setupAdvancedGrid();
    void updateSeriesOptimized();
    void updateSplineSeries();
    void updateSimSeries(); // NUEVO: actualizar serie de simulación

    void initializeMemoryPool();
    void initializeWithOptimizedData();
    void initializeWithZeroData();
    void thinDataPoints();

    void updateHoverDisplay();
    void updatePersistentHover();
    void cleanupHoverElements();
    void createHoverElements(QGraphicsScene *scene, const QPointF &chart_pos,
                             const QPoint &scene_pos, double value, double time);

    double findValueAtScenePosition(const QPoint &scene_pos);
    double interpolateValue(double target_time) const;
    QPointF findNearestPointOnCurve(double absolute_target_time) const;

    std::vector<QPointF> calculateSplinePoints(const std::vector<QPointF> &control_points) const;
    std::vector<double> calculateSplineCoefficients(const std::vector<double> &x,
                                                    const std::vector<double> &y) const;

    ChartConfig config_;
    double start_time_;

    // Chart components
    QChart *chart_;
    HoverChartView *chart_view_;
    QLineSeries *main_series_;
    QSplineSeries *trend_series_;
    QSplineSeries *smooth_series_;

    // NUEVO: Series para datos de simulación
    QLineSeries *sim_main_series_;
    QSplineSeries *sim_smooth_series_;

    QValueAxis *x_axis_;
    QValueAxis *y_axis_;

    std::vector<QGraphicsLineItem *> minor_grid_lines_x_;
    std::vector<QGraphicsLineItem *> minor_grid_lines_y_;

    // Data storage
    std::deque<DataPoint> data_points_;
    std::deque<DataPoint> sim_data_points_; // NUEVO: datos de simulación
    QMutex *data_mutex_;

    // Pre-allocated memory pools
    QVector<QPointF> series_points_buffer_;
    QVector<QPointF> spline_points_buffer_;
    QVector<QPointF> sim_series_points_buffer_; // NUEVO: buffer para simulación
    QVector<QPointF> sim_spline_points_buffer_; // NUEVO
    std::array<DataPoint, 2000> data_pool_;
    size_t pool_write_index_;
    size_t optimal_buffer_size_;

    QTimer *update_timer_;
    double last_update_time_;
    double last_hover_update_time_;
    bool update_pending_;
    bool hover_update_pending_;
    bool data_initialized_;

    QPoint current_scene_pos_;
    QPointF current_chart_pos_;
    QPointF snapped_chart_pos_;
    bool hover_active_;
    bool hover_persistent_;
    double hover_fixed_time_absolute_;
    int hover_fixed_scene_x_ = -1;
    double last_mouse_time_;
    QTimer *hover_timer_;
    double cached_hover_value_;
    double cached_hover_time_;

private:
    void createLayout();
    void setupAxes();
    void setupSeries();

    QPointF sceneToChartCoords(const QPoint &scene_pos) const;
    QPoint chartToSceneCoords(const QPointF &chart_pos) const;
};
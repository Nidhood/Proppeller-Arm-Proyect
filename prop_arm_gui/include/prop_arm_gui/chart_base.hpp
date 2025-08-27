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

    // Public members for hover state tracking
    QPointF hover_position_chart_; // Position in chart coordinates
    QPoint hover_position_scene_;  // Position in scene coordinates
    bool hover_enabled_;
    bool mouse_in_plot_area_;

    void clearHoverElements();

protected:
    void mouseMoveEvent(QMouseEvent *event) override;
    void leaveEvent(QEvent *event) override;

    // Check if mouse is in plot area (grid area only)
    bool isInPlotArea(const QPoint &pos) const;

signals:
    void hoverUpdate(QPointF chart_position, QPoint scene_position, bool valid);

private:
public:
    // Hover elements - public for ChartBase access
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
        double y_min;
        double y_max;
        bool auto_scale;
        bool show_grid;
        double time_window_sec;
        size_t max_points;
        bool show_milliseconds;
        bool use_smooth_curves = true; // Enable cubic spline smoothing
        bool show_minor_grid = true;   // Enable minor grid lines
    };

    explicit ChartBase(const ChartConfig &config, QWidget *parent = nullptr);
    virtual ~ChartBase();

    // Data management - optimized for real-time updates
    void addDataPoint(double value, double timestamp = -1);
    void clearData();
    void setTimeWindow(double seconds);
    void setYRange(double min, double max);
    void setAutoScale(bool enabled);

    // Visual configuration
    void updateTheme();
    void setGridVisible(bool visible);
    void setShowMilliseconds(bool show_ms);
    void setSmoothCurves(bool enabled); // Control spline smoothing

    // Updated neon color scheme
    static const QColor BACKGROUND_DARK;
    static const QColor PLOT_BACKGROUND;
    static const QColor GRID_PRIMARY;
    static const QColor GRID_SECONDARY;
    static const QColor GRID_MINOR; // Minor grid color
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

    // Advanced grid system with major and minor grids
    void setupAdvancedGrid();

    // Series management with spline smoothing
    void updateSeriesOptimized();
    void updateSplineSeries(); // Spline curve update

    // Memory management methods - highly optimized
    void initializeMemoryPool();
    void initializeWithOptimizedData();
    void initializeWithZeroData(); // Initialize with zeros for complete line
    void thinDataPoints();         // Smart data thinning for performance

    // FIXED: Hover methods with absolute time tracking
    void updateHoverDisplay();
    void updatePersistentHover(); // Update hover with new data
    void cleanupHoverElements();
    void createHoverElements(QGraphicsScene *scene, const QPointF &chart_pos,
                             const QPoint &scene_pos, double value, double time);

    // Value lookup with line snapping
    double findValueAtScenePosition(const QPoint &scene_pos);
    double interpolateValue(double target_time) const;
    QPointF findNearestPointOnCurve(double absolute_target_time) const; // FIXED: Uses absolute time

    // Cubic spline interpolation for smooth curves
    std::vector<QPointF> calculateSplinePoints(const std::vector<QPointF> &control_points) const;
    std::vector<double> calculateSplineCoefficients(const std::vector<double> &x,
                                                    const std::vector<double> &y) const;

    // Configuration
    ChartConfig config_;
    double start_time_;

    // Chart components
    QChart *chart_;
    HoverChartView *chart_view_;
    QLineSeries *main_series_;
    QSplineSeries *trend_series_;
    QSplineSeries *smooth_series_; // Smooth spline series
    QValueAxis *x_axis_;
    QValueAxis *y_axis_;

    // Minor grid lines
    std::vector<QGraphicsLineItem *> minor_grid_lines_x_;
    std::vector<QGraphicsLineItem *> minor_grid_lines_y_;

    // Data storage with memory pool
    std::deque<DataPoint> data_points_;
    QMutex *data_mutex_;

    // Pre-allocated memory pools
    QVector<QPointF> series_points_buffer_; // Reused buffer for series updates
    QVector<QPointF> spline_points_buffer_; // Buffer for spline points
    std::array<DataPoint, 2000> data_pool_; // Pre-allocated data pool
    size_t pool_write_index_;
    size_t optimal_buffer_size_;

    // Update management - optimized timing
    QTimer *update_timer_;
    double last_update_time_;
    double last_hover_update_time_;
    bool update_pending_;
    bool hover_update_pending_;
    bool data_initialized_; // Track if data is initialized with zeros

    // FIXED: Hover state with absolute time tracking
    QPoint current_scene_pos_;  // Current mouse position in scene
    QPointF current_chart_pos_; // Current mouse position in chart coordinates
    QPointF snapped_chart_pos_; // Snapped position on curve
    bool hover_active_;
    bool hover_persistent_;            // Keep hover active for data updates
    double hover_fixed_time_absolute_; // FIXED: Absolute time position for persistent hover
    int hover_fixed_scene_x_ = -1;     // NEW: persistent X in scene pixels
    double last_mouse_time_;           // Track last mouse movement time
    QTimer *hover_timer_;              // Timer to detect mouse stops
    double cached_hover_value_;        // Cached interpolated value
    double cached_hover_time_;         // Cached time for value

private:
    void createLayout();
    void setupAxes();
    void setupSeries();

    // Fast coordinate conversions with caching
    QPointF sceneToChartCoords(const QPoint &scene_pos) const;
    QPoint chartToSceneCoords(const QPointF &chart_pos) const;
};
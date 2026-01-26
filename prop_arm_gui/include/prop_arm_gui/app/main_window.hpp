#pragma once

#include <QElapsedTimer>
#include <QListWidget>
#include <QMainWindow>
#include <QMetaObject>
#include <QSplitter>
#include <QStackedWidget>
#include <QString>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <qtabbar.h>

#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/theme_id.hpp"
#include "prop_arm_gui/core/models/domain.hpp"

class QLabel;

namespace prop_arm::core::services {
class RecorderService;
}

namespace prop_arm::models {
class TelemetryStore;
}

namespace prop_arm::infra::ros {
class LimitsRegistry;
}

namespace prop_arm::services {
class CalibrationService;
}

namespace prop_arm::ui::pages {
class CalibrationPage;
class ControlPanelPage;
class DashboardPage;
}

namespace prop_arm::app {

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

    void setTelemetryStore(prop_arm::models::TelemetryStore* store);
    void setRecorderService(prop_arm::core::services::RecorderService* recorder);
    void setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits);

    // ✅ NEW: inject calibration service
    void setCalibrationService(prop_arm::services::CalibrationService* calibration_service);

    void setConnected(bool connected);
    void setControlMode(const QString& mode);
    void setSystemStatus(const QString& status);

    void onTelemetry(const prop_arm::models::TelemetrySample& sample);

    // ✅ NEW: optional direct live updates from ROS bridge (prevents unused-parameter warnings)
    void setStreamLive(prop_arm::models::Domain domain, bool live);

signals:
    void refAngleChanged(double rad);
    void pwmChanged(std::uint16_t pwm_us);
    void autoModeChanged(bool enabled);
    void stopRequested();

protected:
    void showEvent(QShowEvent* event) override;

private:
    void buildUi_();
    void buildTopBar_();
    void connectPageSignals_();
    void setupStatusBar_();
    void setupScreenTracking_();
    void updateModeIndicator_(bool auto_mode);

    void onNavChanged_(int row);
    void onThemeAction_();
    void onThemeChanged_(prop_arm::models::ThemeId id);

    void applyResponsiveInitialGeometry_();
    void clampToCurrentScreen_();

    void updateStatusBarStyle_();
    void updateStatusLabels_();
    void updateTopBarStyle_();

private:
    QSplitter* splitter_{nullptr};
    QListWidget* nav_{nullptr};
    QStackedWidget* stack_{nullptr};
    QToolBar* topbar_{nullptr};
    QToolButton* theme_btn_{nullptr};
    QTabBar* top_tabs_{nullptr};

    prop_arm::ui::pages::DashboardPage* dashboard_{nullptr};
    prop_arm::ui::pages::ControlPanelPage* control_{nullptr};
    prop_arm::ui::pages::CalibrationPage* calibration_{nullptr};

    prop_arm::models::TelemetryStore* store_{nullptr};
    prop_arm::core::services::RecorderService* recorder_{nullptr};
    const prop_arm::infra::ros::LimitsRegistry* limits_{nullptr};
    prop_arm::services::CalibrationService* calibration_service_{nullptr};

    QLabel* status_connection_{nullptr};
    QLabel* status_mode_{nullptr};
    QLabel* status_real_{nullptr};
    QLabel* status_sim_{nullptr};
    QTimer* status_timer_{nullptr};

    bool overall_connected_{false};
    bool seen_real_{false};
    bool seen_sim_{false};
    QElapsedTimer real_timer_;
    QElapsedTimer sim_timer_;

    bool initial_geometry_applied_{false};
    QMetaObject::Connection avail_geom_conn_;
};

} // namespace prop_arm::app

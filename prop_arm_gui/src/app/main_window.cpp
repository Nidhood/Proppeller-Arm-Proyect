#include "prop_arm_gui/app/main_window.hpp"
#include "prop_arm_gui/core/models/domain.hpp"
#include "prop_arm_gui/core/models/theme_id.hpp"
#include "prop_arm_gui/ui/style/theme_manager.hpp"
#include "prop_arm_gui/ui/pages/calibration_page.hpp"
#include "prop_arm_gui/ui/pages/control_panel_page.hpp"
#include "prop_arm_gui/ui/pages/dashboard_page.hpp"
#include <algorithm>
#include <QAction>
#include <QApplication>
#include <QCursor>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QScreen>
#include <QShowEvent>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStyle>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>
#include <qnamespace.h>
#include <qsizepolicy.h>
#include <qtabbar.h>
#include <qwidget.h>
namespace prop_arm::app {
using prop_arm::models::Domain;
namespace {
// If no data arrives within this window, the stream is considered "stale".
constexpr int kStreamStaleMs = 1000;
} // namespace
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setWindowTitle("PropArm HMI");
    QIcon app_icon(":/icons/prop_arm_icon.png");
    if (!app_icon.isNull()) {
        setWindowIcon(app_icon);
        QApplication::setWindowIcon(app_icon);
    } else {
        qWarning() << "Failed to load application icon from resources";
    }
    setMinimumSize(1200, 760);
    buildUi_();
    buildTopBar_();
    connectPageSignals_();
    setupStatusBar_();
    setupScreenTracking_();
    // Connect to theme manager to update status bar when theme changes
    connect(&prop_arm::ui::style::ThemeManager::instance(),
            &prop_arm::ui::style::ThemeManager::themeChanged,
            this, &MainWindow::onThemeChanged_);
    if (top_tabs_) top_tabs_->setCurrentIndex(0);
    onNavChanged_(0);
}
void MainWindow::buildUi_() {
    auto* central = new QWidget(this);
    setCentralWidget(central);
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    // Get pages:
    stack_ = new QStackedWidget(central);
    stack_->setObjectName("stack");
    dashboard_   = new prop_arm::ui::pages::DashboardPage(stack_);
    control_     = new prop_arm::ui::pages::ControlPanelPage(stack_);
    calibration_ = new prop_arm::ui::pages::CalibrationPage(stack_);
    stack_->addWidget(dashboard_);
    stack_->addWidget(control_);
    stack_->addWidget(calibration_);
    root->addWidget(stack_, 1);
}
void MainWindow::buildTopBar_() {
    topbar_ = new QToolBar(this);
    topbar_->setObjectName("topbar");
    topbar_->setMovable(false);
    topbar_->setFloatable(false);
    topbar_->setIconSize(QSize(18, 18));
    theme_btn_ = new QToolButton(this);
    theme_btn_->setObjectName("themeButton");
    theme_btn_->setAutoRaise(true);
    theme_btn_->setPopupMode(QToolButton::InstantPopup);
    theme_btn_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    theme_btn_->setIcon(style()->standardIcon(QStyle::SP_DesktopIcon));
    theme_btn_->setText("Theme");
    auto* themeMenu = new QMenu(theme_btn_);
    themeMenu->setObjectName("themeMenu");
    auto addTheme = [&](const QString& name, prop_arm::models::ThemeId id) {
        QAction* a = themeMenu->addAction(name);
        a->setData(static_cast<int>(id));
        connect(a, &QAction::triggered, this, &MainWindow::onThemeAction_);
    };
    addTheme("Light", prop_arm::models::ThemeId::Light);
    addTheme("Dark", prop_arm::models::ThemeId::Dark);
    addTheme("MATLAB", prop_arm::models::ThemeId::MATLAB);
    addTheme("Tron Evolution", prop_arm::models::ThemeId::TronEvolution);
    addTheme("Tron Ares", prop_arm::models::ThemeId::TronAres);
    addTheme("Cyberpunk Neon", prop_arm::models::ThemeId::CyberpunkNeon);
    theme_btn_->setMenu(themeMenu);
    // Navigation tabs:
    top_tabs_ = new QTabBar(this);
    top_tabs_->setObjectName("topTabs");
    top_tabs_->setExpanding(false);
    top_tabs_->setMovable(false);
    top_tabs_->setElideMode(Qt::ElideNone);
    top_tabs_->setUsesScrollButtons(false);
    top_tabs_->setDrawBase(false);  // Remove the base line under tabs
    top_tabs_->addTab("Dashboard");
    top_tabs_->addTab("Control Panel");
    top_tabs_->addTab("Calibration");
    connect(top_tabs_, &QTabBar::currentChanged, this, [this](int idx) {
        onNavChanged_(idx);
    });
    addToolBar(Qt::TopToolBarArea, topbar_);
    // Add tabs first on the left
    topbar_->addWidget(top_tabs_);
    // Push theme button to the right:
    auto* spacer = new QWidget(this);
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    topbar_->addWidget(spacer);
    // Theme button on the extreme right
    topbar_->addWidget(theme_btn_);
    // Apply initial topbar styling (must be after adding widgets)
    updateTopBarStyle_();
}
void MainWindow::onThemeAction_() {
    auto* a = qobject_cast<QAction*>(sender());
    if (!a) return;
    const auto id = static_cast<prop_arm::models::ThemeId>(a->data().toInt());
    prop_arm::ui::style::ThemeManager::instance().apply(id);
}
void MainWindow::connectPageSignals_() {
    connect(control_, &prop_arm::ui::pages::ControlPanelPage::refAngleChanged,
            this, &MainWindow::refAngleChanged);
    connect(control_, &prop_arm::ui::pages::ControlPanelPage::pwmChanged,
            this, &MainWindow::pwmChanged);
    connect(control_, &prop_arm::ui::pages::ControlPanelPage::autoModeChanged,
    this, [this](bool auto_mode) {
        updateModeIndicator_(auto_mode);
        emit autoModeChanged(auto_mode);
    });
    connect(control_, &prop_arm::ui::pages::ControlPanelPage::stopRequested,
            this, &MainWindow::stopRequested);
}
void MainWindow::setupStatusBar_() {
    // Create modern styled status labels with separators
    status_connection_ = new QLabel("Disconnected");
    status_connection_->setObjectName("statusLabel");
    status_mode_ = new QLabel("Mode: Manual");
    status_mode_->setObjectName("statusLabel");
    status_real_ = new QLabel("REAL: --");
    status_real_->setObjectName("statusLabel");
    status_sim_ = new QLabel("SIM: --");
    status_sim_->setObjectName("statusLabel");
    // Create separator labels
    auto createSeparator = [this]() {
        auto* sep = new QLabel(" | ");
        sep->setObjectName("statusSeparator");
        return sep;
    };
    statusBar()->setObjectName("statusBar");
    statusBar()->setSizeGripEnabled(false);
    statusBar()->addPermanentWidget(status_connection_);
    statusBar()->addPermanentWidget(createSeparator());
    statusBar()->addPermanentWidget(status_mode_);
    statusBar()->addPermanentWidget(createSeparator());
    statusBar()->addPermanentWidget(status_real_);
    statusBar()->addPermanentWidget(createSeparator());
    statusBar()->addPermanentWidget(status_sim_);
    // Apply initial styling
    updateStatusBarStyle_();
    // Watchdog to update REAL/SIM status when streams go stale
    status_timer_ = new QTimer(this);
    status_timer_->setInterval(200);
    status_timer_->setTimerType(Qt::CoarseTimer);
    connect(status_timer_, &QTimer::timeout, this, &MainWindow::updateStatusLabels_);
    status_timer_->start();
}
void MainWindow::updateStatusBarStyle_() {
    const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();
    // Build dynamic stylesheet with !important and inline format to override global theme
    QString statusBarStyle = QString(
                                 "QStatusBar#statusBar { "
                                 "background-color: %1 !important; "
                                 "color: %2 !important; "
                                 "border-top: 1px solid %3 !important; "
                                 "padding: 7px 20px !important; "
                                 "min-height: 20px !important; "
                                 "} "
                                 "QLabel#statusLabel { "
                                 "color: %2 !important; "
                                 "font-weight: 600 !important; "
                                 "padding: 2px 8px !important; "
                                 "background-color: transparent !important; "
                                 "} "
                                 "QLabel#statusSeparator { "
                                 "color: %4 !important; "
                                 "font-weight: 400 !important; "
                                 "background-color: transparent !important; "
                                 "} "
                             )
                             .arg(spec.panel.name())
                             .arg(spec.text.name())
                             .arg(spec.accent.name())
                             .arg(spec.text_muted.name());
    statusBar()->setStyleSheet(statusBarStyle);
}
void MainWindow::updateTopBarStyle_() {
    const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();
    // Use very specific selectors and !important to override global QSS
    QString topBarStyle = QString(
                              "QToolBar#topbar { "
                              "background-color: %1 !important; "
                              "border: none !important; "
                              "border-bottom: 1px solid %2 !important; "
                              "padding: 0px !important; "
                              "spacing: 0px !important; "
                              "min-height: 40px !important; "
                              "} "
                              "QToolBar#topbar::separator { "
                              "background: transparent !important; "
                              "width: 0px !important; "
                              "} "
                              "QToolButton#themeButton { "
                              "background-color: transparent !important; "
                              "color: %3 !important; "
                              "border: none !important; "
                              "padding: 8px 12px !important; "
                              "border-radius: 4px !important; "
                              "} "
                              "QToolButton#themeButton:hover { "
                              "background-color: %4 !important; "
                              "color: %5 !important; "
                              "} "
                              "QToolButton#themeButton::menu-indicator { "
                              "image: none !important; "
                              "} "
                          )
                          .arg(spec.panel.name())
                          .arg(spec.accent.name())
                          .arg(spec.text.name())
                          .arg(spec.accent.lighter(130).name())
                          .arg(spec.bg.name());
    topbar_->setStyleSheet(topBarStyle);
    QString tabBarStyle = QString(
                              "QTabBar#topTabs { "
                              "background: transparent !important; "
                              "border: none !important; "
                              "} "
                              "QTabBar#topTabs::tab { "
                              "background: transparent !important; "
                              "color: %1 !important; "
                              "border: none !important; "
                              "padding: 10px 20px !important; "
                              "margin: 0px !important; "
                              "font-weight: 600 !important; "
                              "} "
                              "QTabBar#topTabs::tab:hover { "
                              "background: %2 !important; "
                              "color: %3 !important; "
                              "font-weight: 400 !important; "
                              "border-top-left-radius: 8px !important; "
                              "border-top-right-radius: 8px !important; "
                              "} "
                              "QTabBar#topTabs::tab:selected { "
                              "background: %4 !important; "
                              "color: %2 !important; "
                              "font-weight: 600 !important; "
                              "border: 1px solid %2 !important; "
                              "border-bottom: 3px solid %4 !important; "
                              "border-top-left-radius: 8px !important; "
                              "border-top-right-radius: 8px !important; "
                              "margin-bottom: -2px !important; "
                              "padding-bottom: 9px !important; "
                              "} "
                          )
                          .arg(spec.text_muted.name())
                          .arg(spec.accent.name())
                          .arg(spec.bg.name())
                          .arg(spec.bg.name());
    top_tabs_->setStyleSheet(tabBarStyle);

    // Style the theme menu with accent color on hover
    QString menuStyle = QString(
                            "QMenu#themeMenu { "
                            "background-color: %1 !important; "
                            "color: %2 !important; "
                            "border: 1px solid %3 !important; "
                            "padding: 5px !important; "
                            "} "
                            "QMenu#themeMenu::item { "
                            "background-color: transparent !important; "
                            "color: %2 !important; "
                            "padding: 8px 20px !important; "
                            "border-radius: 4px !important; "
                            "} "
                            "QMenu#themeMenu::item:hover { "
                            "background-color: %3 !important; "
                            "color: %4 !important; "
                            "} "
                            "QMenu#themeMenu::item:selected { "
                            "background-color: %3 !important; "
                            "color: %4 !important; "
                            "} "
                        )
                        .arg(spec.panel.name())
                        .arg(spec.text.name())
                        .arg(spec.accent.name())
                        .arg(spec.bg.name());

    if (theme_btn_ && theme_btn_->menu()) {
        theme_btn_->menu()->setStyleSheet(menuStyle);
    }
}
void MainWindow::updateModeIndicator_(bool auto_mode) {
    if (!status_mode_) return;
    const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();
    if (auto_mode) {
        status_mode_->setText("Mode: Auto");
        // Auto mode uses accent color (like checked button)
        status_mode_->setStyleSheet(QString("color: %1 !important; font-weight: bold !important; background: transparent !important;").arg(spec.accent.name()));
    } else {
        status_mode_->setText("Mode: Manual");
        // Manual mode uses muted text color (like unchecked button)
        status_mode_->setStyleSheet(QString("color: %1 !important; font-weight: 600 !important; background: transparent !important;").arg(spec.text_muted.name()));
    }
}
void MainWindow::updateStatusLabels_() {
    const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();
    // Define color states based on theme
    const QColor colorOk = spec.accent2;      // Green/success accent
    const QColor colorBad = QColor("#FF6B6B"); // Universal red for errors
    const QColor colorWarning = QColor("#FFA500"); // Universal orange for warnings
    const QColor colorMuted = spec.text_muted;
    // Update connection status
    if (overall_connected_) {
        status_connection_->setText("Connected");
        status_connection_->setStyleSheet(QString("color: %1 !important; font-weight: bold !important; background: transparent !important;").arg(colorOk.name()));
    } else {
        status_connection_->setText("Disconnected");
        status_connection_->setStyleSheet(QString("color: %1 !important; font-weight: bold !important; background: transparent !important;").arg(colorBad.name()));
    }
    // Mode status styling is handled by updateModeIndicator_() when mode changes
    // but we keep the current style here
    // REAL stream status
    bool real_live = false;
    if (!seen_real_) {
        status_real_->setText("REAL: --");
        status_real_->setStyleSheet(QString("color: %1 !important; background: transparent !important;").arg(colorMuted.name()));
    } else {
        const qint64 ms = real_timer_.elapsed();
        if (ms <= kStreamStaleMs) {
            status_real_->setText("REAL: LIVE");
            status_real_->setStyleSheet(QString("color: %1 !important; font-weight: bold !important; background: transparent !important;").arg(colorOk.name()));
            real_live = true;
        } else {
            status_real_->setText("REAL: STALE");
            status_real_->setStyleSheet(QString("color: %1 !important; font-weight: bold !important; background: transparent !important;").arg(colorWarning.name()));
        }
    }
    // SIM stream status
    bool sim_live = false;
    if (!seen_sim_) {
        status_sim_->setText("SIM: --");
        status_sim_->setStyleSheet(QString("color: %1 !important; background: transparent !important;").arg(colorMuted.name()));
    } else {
        const qint64 ms = sim_timer_.elapsed();
        if (ms <= kStreamStaleMs) {
            status_sim_->setText("SIM: LIVE");
            status_sim_->setStyleSheet(QString("color: %1 !important; font-weight: bold !important; background: transparent !important;").arg(colorOk.name()));
            sim_live = true;
        } else {
            status_sim_->setText("SIM: STALE");
            status_sim_->setStyleSheet(QString("color: %1 !important; font-weight: bold !important; background: transparent !important;").arg(colorWarning.name()));
        }
    }
    // Update chart legends based on live status
    if (dashboard_) {
        dashboard_->setStreamLive(prop_arm::models::Domain::Real, real_live);
        dashboard_->setStreamLive(prop_arm::models::Domain::Sim, sim_live);
    }
    if (control_) {
        control_->setStreamLive(prop_arm::models::Domain::Real, real_live);
        control_->setStreamLive(prop_arm::models::Domain::Sim, sim_live);
    }
    if (calibration_) {
        calibration_->setStreamLive(prop_arm::models::Domain::Real, real_live);
        calibration_->setStreamLive(prop_arm::models::Domain::Sim, sim_live);
    }
}
void MainWindow::onThemeChanged_(prop_arm::models::ThemeId id) {
    Q_UNUSED(id);
    // Reapply our custom styles after theme change
    QTimer::singleShot(10, this, [this]() {
        const auto& spec = prop_arm::ui::style::ThemeManager::instance().currentSpec();

        // Get current application stylesheet
        QString appStyle = qApp->styleSheet();

        // Append our custom overrides to the global stylesheet
        QString customOverrides = QString(
                                      "QToolBar#topbar { "
                                      "background-color: %1 !important; "
                                      "border: none !important; "
                                      "border-bottom: 1px solid %2 !important; "
                                      "padding: 0px !important; "
                                      "spacing: 0px !important; "
                                      "min-height: 40px !important; "
                                      "} "
                                      "QStatusBar#statusBar { "
                                      "background-color: %1 !important; "
                                      "color: %3 !important; "
                                      "border-top: 1px solid %2 !important; "
                                      "padding: 7px 20px !important; "
                                      "min-height: 20px !important; "
                                      "} "
                                  )
                                  .arg(spec.panel.name())
                                  .arg(spec.accent.name())
                                  .arg(spec.text.name());

        // Apply the combined stylesheet
        qApp->setStyleSheet(appStyle + "\n" + customOverrides);

        // Now apply local styles
        updateTopBarStyle_();
        updateStatusBarStyle_();
        updateStatusLabels_();

        // Force widget updates
        if (topbar_) topbar_->update();
        if (top_tabs_) top_tabs_->update();
        if (statusBar()) statusBar()->update();
    });
}
void MainWindow::onNavChanged_(int row) {
    if (!stack_) return;
    row = std::clamp(row, 0, stack_->count() - 1);
    stack_->setCurrentIndex(row);
    if (top_tabs_ && top_tabs_->currentIndex() != row) {
        top_tabs_->setCurrentIndex(row);
    }
    if (top_tabs_) {
        top_tabs_->update();
        top_tabs_->repaint();
    }
}
void MainWindow::setTelemetryStore(prop_arm::models::TelemetryStore* store) {
    store_ = store;
    if (dashboard_) dashboard_->setTelemetryStore(store_);
}
void MainWindow::setRecorderService(prop_arm::core::services::RecorderService* recorder) {
    recorder_ = recorder;
    if (control_) {
        control_->setRecorderService(recorder_);
    }
}
void MainWindow::setConnected(bool connected) {
    overall_connected_ = connected;
    updateStatusLabels_();
}
void MainWindow::setControlMode(const QString& mode) {
    status_mode_->setText(QString("Mode: %1").arg(mode));
}
void MainWindow::setSystemStatus(const QString& status) {
    // This can be extended if you want a separate system status label
    Q_UNUSED(status);
}
void MainWindow::onTelemetry(const prop_arm::models::TelemetrySample& sample) {
    // Forward to pages
    if (dashboard_) dashboard_->onTelemetry(sample);
    if (control_)   control_->onTelemetry(sample);
    if (calibration_) calibration_->onTelemetry(sample);
    // Track stream activity (REAL vs SIM)
    if (sample.domain == Domain::Real) {
        if (!seen_real_) {
            seen_real_ = true;
            real_timer_.start();
        } else {
            real_timer_.restart();
        }
    } else {
        if (!seen_sim_) {
            seen_sim_ = true;
            sim_timer_.start();
        } else {
            sim_timer_.restart();
        }
    }
}
void MainWindow::showEvent(QShowEvent* event) {
    QMainWindow::showEvent(event);
    // Apply once, after Qt has a real window handle/screen.
    if (!initial_geometry_applied_) {
        initial_geometry_applied_ = true;
        applyResponsiveInitialGeometry_();
    }
}
void MainWindow::applyResponsiveInitialGeometry_() {
    QScreen* screen = nullptr;
    if (windowHandle() && windowHandle()->screen()) {
        screen = windowHandle()->screen();
    } else {
        screen = QGuiApplication::screenAt(QCursor::pos());
        if (!screen) screen = QGuiApplication::primaryScreen();
    }
    if (!screen) return;
    const QRect avail = screen->availableGeometry();
    // Responsive rule: target 92% of available size (taskbar/docks aware).
    const int targetW = static_cast<int>(avail.width()  * 0.92);
    const int targetH = static_cast<int>(avail.height() * 0.92);
    const QSize minSz = minimumSize();
    const int w = std::max(minSz.width(),  std::min(targetW, avail.width()));
    const int h = std::max(minSz.height(), std::min(targetH, avail.height()));
    // Center within that screen.
    const int x = avail.x() + (avail.width()  - w) / 2;
    const int y = avail.y() + (avail.height() - h) / 2;
    setGeometry(QRect(QPoint(x, y), QSize(w, h)));
}
void MainWindow::clampToCurrentScreen_() {
    QScreen* screen = nullptr;
    if (windowHandle() && windowHandle()->screen()) {
        screen = windowHandle()->screen();
    } else {
        screen = QGuiApplication::primaryScreen();
    }
    if (!screen) return;
    const QRect avail = screen->availableGeometry();
    QRect g = geometry();
    // Clamp size
    const QSize minSz = minimumSize();
    const int w = std::max(minSz.width(),  std::min(g.width(),  avail.width()));
    const int h = std::max(minSz.height(), std::min(g.height(), avail.height()));
    g.setSize(QSize(w, h));
    // Clamp position (keep the whole window visible)
    if (g.left() < avail.left())     g.moveLeft(avail.left());
    if (g.top() < avail.top())       g.moveTop(avail.top());
    if (g.right() > avail.right())   g.moveRight(avail.right());
    if (g.bottom() > avail.bottom()) g.moveBottom(avail.bottom());
    setGeometry(g);
}
void MainWindow::setupScreenTracking_() {
    // Defer until the native window exists (windowHandle becomes valid).
    QTimer::singleShot(0, this, [this]() {
        QWindow* w = windowHandle();
        if (!w) return;
        auto attachAvailableGeometryListener = [this](QScreen* s) {
            if (!s) return;
            // Avoid duplicates
            if (avail_geom_conn_) {
                QObject::disconnect(avail_geom_conn_);
                avail_geom_conn_ = {};
            }
            avail_geom_conn_ = connect(
                                   s, &QScreen::availableGeometryChanged,
            this, [this](const QRect&) {
                clampToCurrentScreen_();
            }
                               );
        };
        // Initial attach for current screen
        attachAvailableGeometryListener(w->screen());
        // When the window moves to another monitor, clamp and re-attach
        connect(w, &QWindow::screenChanged, this,
        [this, attachAvailableGeometryListener](QScreen* s) mutable {
            clampToCurrentScreen_();
            attachAvailableGeometryListener(s);
        });
    });
}
void MainWindow::setLimitsRegistry(const prop_arm::infra::ros::LimitsRegistry* limits) {
    limits_ = limits;
    // Set limits on all pages that have plots
    if (dashboard_) {
        dashboard_->setLimitsRegistry(limits);
    }
    if (control_) {
        control_->setLimitsRegistry(limits);
    }
    if (calibration_) {
        calibration_->setLimitsRegistry(limits);
    }
}
void MainWindow::setCalibrationService(prop_arm::services::CalibrationService* service) {
    calibration_service_ = service;
    if (calibration_) {
        calibration_->setCalibrationService(service);
    }
}
void MainWindow::setStreamLive(prop_arm::models::Domain domain, bool live) {
    if (dashboard_)   dashboard_->setStreamLive(domain, live);
    if (control_)     control_->setStreamLive(domain, live);
    if (calibration_) calibration_->setStreamLive(domain, live);
}
} // namespace prop_arm::app
#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "prop_arm_gui/app/app_context.hpp"
#include "prop_arm_gui/app/ros_qt_bridge.hpp"
#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/theme_id.hpp"
#include "prop_arm_gui/ui/style/theme_manager.hpp"

int main(int argc, char* argv[]) {

    // 1. Qt first (safe argv ownership & GUI prerequisites):
    QApplication app(argc, argv);
    qRegisterMetaType<prop_arm::models::TelemetrySample>("prop_arm::models::TelemetrySample");

    // 2. ROS after Qt:
    rclcpp::init(argc, argv);

    // 3. Theme:
    prop_arm::ui::style::ThemeManager::instance()
    .apply(prop_arm::models::ThemeId::TronAres);

    // 4. App context:
    prop_arm::app::AppContext context;
    context.mainWindow().show();

    // 5. Bridge ROS <-> Qt lifetime:
    prop_arm::app::RosQtBridge bridge(app, context.node());

    // 6. Qt loop:
    return app.exec();
}

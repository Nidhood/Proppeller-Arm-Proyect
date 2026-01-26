#pragma once

#include <QString>
#include <QColor>

#include "prop_arm_gui/core/models/theme_id.hpp"

namespace prop_arm::ui::style {

struct ThemeSpec {
    models::ThemeId id{};
    QString display_name;
    QString qss_path;
    QColor accent;
    QColor accent2;
    QColor ref_color;
    QColor bg;
    QColor panel;
    QColor text;
    QColor text_muted;
};

} // namespace prop_arm::ui::style

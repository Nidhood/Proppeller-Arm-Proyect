#pragma once

#include <QColor>
#include <QObject>
#include <QPalette>
#include <QString>

#include "prop_arm_gui/core/models/theme_id.hpp"
#include "prop_arm_gui/ui/style/theme.hpp"

namespace prop_arm::ui::style {

using prop_arm::ui::style::ThemeSpec;

class ThemeManager : public QObject {
    Q_OBJECT

public:
    static ThemeManager& instance();

    void apply(prop_arm::models::ThemeId id, const QString& override_qss_path = {});

    // Getter for current theme spec
    const ThemeSpec& currentSpec() const {
        return current_spec_;
    }
    prop_arm::models::ThemeId currentId() const {
        return current_id_;
    }

signals:
    void themeChanged(prop_arm::models::ThemeId id);

private:
    explicit ThemeManager(QObject* parent = nullptr);
    ~ThemeManager() override = default;

    ThemeManager(const ThemeManager&) = delete;
    ThemeManager& operator=(const ThemeManager&) = delete;

    ThemeSpec makeSpec_(prop_arm::models::ThemeId id);
    QString loadQss_(const QString& path);
    QPalette buildPalette_(const ThemeSpec& spec);

    prop_arm::models::ThemeId current_id_{prop_arm::models::ThemeId::MATLAB};
    ThemeSpec current_spec_;
};

} // namespace prop_arm::ui::style
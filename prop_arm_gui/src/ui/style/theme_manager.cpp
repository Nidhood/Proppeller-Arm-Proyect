#include "prop_arm_gui/ui/style/theme_manager.hpp"
#include "prop_arm_gui/core/models/theme_id.hpp"

#include <QApplication>
#include <QColor>
#include <QFile>
#include <QPalette>
#include <QTextStream>

namespace prop_arm::ui::style {

ThemeManager& ThemeManager::instance() {
    static ThemeManager inst;
    return inst;
}

ThemeManager::ThemeManager(QObject* parent)
    : QObject(parent), current_spec_(makeSpec_(current_id_)) {}

void ThemeManager::apply(models::ThemeId id, const QString& override_qss_path) {
    ThemeSpec spec = makeSpec_(id);
    if (!override_qss_path.isEmpty()) {
        spec.qss_path = override_qss_path;
    }

    const QString qss = loadQss_(spec.qss_path);
    if (qApp) {
        qApp->setStyleSheet(qss);
        qApp->setPalette(buildPalette_(spec));
        qApp->setStyleSheet(qApp->styleSheet());
    }

    current_id_ = id;
    current_spec_ = spec;
    emit themeChanged(id);
}

ThemeSpec ThemeManager::makeSpec_(models::ThemeId id) {
    switch (id) {
    case models::ThemeId::Light:
        return ThemeSpec{
            .id = id,
            .display_name = "Light",
            .qss_path = ":/theme/light.qss",
            .accent = QColor("#3B82F6"),        // Vibrant blue
            .accent2 = QColor("#10B981"),       // Emerald green
            .ref_color = QColor("#8B5CF6"),     // Purple for reference
            .bg = QColor("#F8FAFC"),
            .panel = QColor("#FFFFFF"),
            .text = QColor("#0F172A"),
            .text_muted = QColor("#64748B")
        };

    case models::ThemeId::Dark:
        return ThemeSpec{
            .id = id,
            .display_name = "Dark",
            .qss_path = ":/theme/dark.qss",
            .accent = QColor("#60A5FA"),        // Light blue
            .accent2 = QColor("#34D399"),       // Emerald
            .ref_color = QColor("#A78BFA"),     // Light purple for reference
            .bg = QColor("#0A0E1A"),
            .panel = QColor("#141B2E"),
            .text = QColor("#E2E8F0"),
            .text_muted = QColor("#94A3B8")
        };

    case models::ThemeId::MATLAB:
        return ThemeSpec{
            .id = id,
            .display_name = "MATLAB",
            .qss_path = ":/theme/matlab.qss",
            .accent = QColor("#0072BD"),        // MATLAB blue
            .accent2 = QColor("#D95319"),       // MATLAB orange
            .ref_color = QColor("#77AC30"),     // MATLAB green
            .bg = QColor("#ECECEC"),
            .panel = QColor("#FAFAFA"),
            .text = QColor("#000000"),
            .text_muted = QColor("#666666")
        };

    case models::ThemeId::CyberpunkNeon:
        return ThemeSpec{
            .id = id,
            .display_name = "Cyberpunk Neon",
            .qss_path = ":/theme/cyberpunk.qss",
            .accent = QColor("#FF2BD6"),        // Hot pink
            .accent2 = QColor("#00F5FF"),       // Cyan
            .ref_color = QColor("#CCFF00"),     // Neon yellow-green
            .bg = QColor("#050510"),
            .panel = QColor("#0B0B18"),
            .text = QColor("#EAF2FF"),
            .text_muted = QColor("#8AA0C8")
        };

    case models::ThemeId::TronAres:
        return ThemeSpec{
            .id = id,
            .display_name = "Tron Ares",
            .qss_path = ":/theme/tron_ares.qss",
            .accent = QColor("#FF0033"),        // Ares red
            .accent2 = QColor("#3399FF"),       // Tron blue
            .ref_color = QColor("#00E6FF"),     // Cyan for reference
            .bg = QColor("#000000"),
            .panel = QColor("#0A0A14"),
            .text = QColor("#E0E8F0"),
            .text_muted = QColor("#5588BB")
        };

    case models::ThemeId::TronEvolution:
        return ThemeSpec{
            .id = id,
            .display_name = "Tron Evolution",
            .qss_path = ":/theme/tron_evolution.qss",
            .accent = QColor("#00E6FF"),        // Bright cyan
            .accent2 = QColor("#B084FF"),       // Purple (distinct!)
            .ref_color = QColor("#00FF99"),     // Cyan-green for reference
            .bg = QColor("#000000"),
            .panel = QColor("#000F1A"),
            .text = QColor("#D0F0FF"),
            .text_muted = QColor("#7BB8DD")
        };

    default:
        return ThemeSpec{
            .id = id,
            .display_name = "MATLAB",
            .qss_path = ":/theme/matlab.qss",
            .accent = QColor("#0072BD"),
            .accent2 = QColor("#D95319"),
            .ref_color = QColor("#00E6FF"),
            .bg = QColor("#ECECEC"),
            .panel = QColor("#FAFAFA"),
            .text = QColor("#000000"),
            .text_muted = QColor("#666666")
        };
    }
}

QString ThemeManager::loadQss_(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }
    QTextStream in(&file);
    return in.readAll();
}

QPalette ThemeManager::buildPalette_(const ThemeSpec& spec) {
    QPalette p;

    p.setColor(QPalette::Window, spec.bg);
    p.setColor(QPalette::WindowText, spec.text);
    p.setColor(QPalette::Base, spec.panel);
    p.setColor(QPalette::AlternateBase, spec.bg.lighter(105));

    p.setColor(QPalette::Text, spec.text);
    p.setColor(QPalette::BrightText, spec.text.lighter(110));
    p.setColor(QPalette::ToolTipBase, spec.panel.lighter(110));
    p.setColor(QPalette::ToolTipText, spec.text);

    p.setColor(QPalette::Button, spec.panel);
    p.setColor(QPalette::ButtonText, spec.text);

    p.setColor(QPalette::Highlight, spec.accent);
    p.setColor(QPalette::HighlightedText, QColor("#FFFFFF"));

    p.setColor(QPalette::Link, spec.accent2);
    p.setColor(QPalette::LinkVisited, spec.accent);

    QColor disabledText = spec.text_muted;
    disabledText.setAlpha(140);
    p.setColor(QPalette::Disabled, QPalette::Text, disabledText);
    p.setColor(QPalette::Disabled, QPalette::WindowText, disabledText);
    p.setColor(QPalette::Disabled, QPalette::ButtonText, disabledText);

    return p;
}

} // namespace prop_arm::ui::style
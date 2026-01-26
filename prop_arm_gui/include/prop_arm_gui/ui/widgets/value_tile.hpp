#pragma once

#include <QFrame>
#include <QString>

class QLabel;

namespace prop_arm::ui::widgets {

class ValueTile final : public QFrame {
    Q_OBJECT

public:
    explicit ValueTile(const QString& label = {},
                       const QString& unit = {},
                       QWidget* parent = nullptr);

    void setLabel(const QString& label);
    void setUnit(const QString& unit);
    void setValueText(const QString& text);

    // Convenience for numeric values
    void setValue(double value, int decimals = 2);

private:
    QLabel* label_;
    QLabel* value_;
    QLabel* unit_;
};

} // namespace prop_arm::ui::widgets

#pragma once

#include <QWidget>
#include <QString>

class QLabel;

namespace prop_arm::ui::widgets {

class SectionHeader final : public QWidget {
    Q_OBJECT

public:
    explicit SectionHeader(const QString& title = {},
                           const QString& subtitle = {},
                           QWidget* parent = nullptr);

    void setTitle(const QString& title);
    void setSubtitle(const QString& subtitle);

private:
    QLabel* title_;
    QLabel* subtitle_;
};

} // namespace prop_arm::ui::widgets

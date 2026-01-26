#pragma once

#include <QFrame>

class QLabel;
class QVBoxLayout;

namespace prop_arm::ui::widgets {

class ChartFrame final : public QFrame {
    Q_OBJECT

public:
    explicit ChartFrame(QWidget* parent = nullptr);

    void setTitle(const QString& title);
    void setChartWidget(QWidget* w);

private:
    QLabel* title_{nullptr};
    QVBoxLayout* root_{nullptr};
    QWidget* chart_{nullptr};
};

}  // namespace prop_arm::ui::widgets

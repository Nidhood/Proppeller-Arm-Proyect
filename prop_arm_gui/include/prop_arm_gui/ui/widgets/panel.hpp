#pragma once

#include <QFrame>
#include <QString>

class QVBoxLayout;
class QLabel;

namespace prop_arm::ui::widgets {

class Panel final : public QFrame {
    Q_OBJECT

public:
    explicit Panel(const QString& title = {}, QWidget* parent = nullptr);

    QVBoxLayout* bodyLayout() const noexcept {
        return body_;
    }
    void setTitle(const QString& title);
    QString title() const;

private:
    QLabel* title_;
    QVBoxLayout* root_;
    QVBoxLayout* body_;
};

} // namespace prop_arm::ui::widgets

#include "prop_arm_gui/ui/widgets/panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace prop_arm::ui::widgets {

Panel::Panel(const QString& title, QWidget* parent)
    : QFrame(parent),
      title_(new QLabel(this)),
      root_(new QVBoxLayout(this)),
      body_(new QVBoxLayout()) {

    setObjectName("panel");
    setFrameShape(QFrame::NoFrame);

    root_->setContentsMargins(12, 12, 12, 12);
    root_->setSpacing(10);

    title_->setObjectName("panelTitle");
    title_->setText(title);
    title_->setVisible(!title.isEmpty());

    root_->addWidget(title_);
    root_->addLayout(body_);
}

void Panel::setTitle(const QString& title) {
    title_->setText(title);
    title_->setVisible(!title.isEmpty());
}

QString Panel::title() const {
    return title_->text();
}

} // namespace prop_arm::ui::widgets

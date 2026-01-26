#include "prop_arm_gui/ui/widgets/section_header.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace prop_arm::ui::widgets {

SectionHeader::SectionHeader(const QString& title, const QString& subtitle, QWidget* parent)
    : QWidget(parent),
      title_(new QLabel(this)),
      subtitle_(new QLabel(this)) {

    setObjectName("sectionHeader");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(2);

    title_->setObjectName("title");
    title_->setText(title);

    subtitle_->setObjectName("subtitle");
    subtitle_->setText(subtitle);
    subtitle_->setVisible(!subtitle.isEmpty());

    root->addWidget(title_);
    root->addWidget(subtitle_);
}

void SectionHeader::setTitle(const QString& title) {
    title_->setText(title);
}

void SectionHeader::setSubtitle(const QString& subtitle) {
    subtitle_->setText(subtitle);
    subtitle_->setVisible(!subtitle.isEmpty());
}

} // namespace prop_arm::ui::widgets

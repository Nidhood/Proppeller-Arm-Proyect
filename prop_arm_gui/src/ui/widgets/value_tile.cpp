#include "prop_arm_gui/ui/widgets/value_tile.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QVBoxLayout>

namespace prop_arm::ui::widgets {

ValueTile::ValueTile(const QString& label, const QString& unit, QWidget* parent)
    : QFrame(parent),
      label_(new QLabel(this)),
      value_(new QLabel(this)),
      unit_(new QLabel(this)) {

    setObjectName("valueTile");
    setFrameShape(QFrame::NoFrame);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 10, 12, 10);
    root->setSpacing(6);

    label_->setObjectName("valueTileLabel");
    label_->setText(label);

    value_->setObjectName("valueTileValue");
    value_->setText("--");

    unit_->setObjectName("valueTileUnit");
    unit_->setText(unit);

    auto* row = new QHBoxLayout();
    row->setContentsMargins(0, 0, 0, 0);
    row->setSpacing(8);
    row->addWidget(value_, 0, Qt::AlignLeft);
    row->addWidget(unit_, 0, Qt::AlignLeft);
    row->addStretch(1);

    root->addWidget(label_);
    root->addLayout(row);
}

void ValueTile::setLabel(const QString& label) {
    label_->setText(label);
}
void ValueTile::setUnit(const QString& unit) {
    unit_->setText(unit);
}
void ValueTile::setValueText(const QString& text) {
    value_->setText(text);
}

void ValueTile::setValue(double v, int decimals) {
    const QLocale loc = QLocale::c();
    value_->setText(loc.toString(v, 'f', decimals));
}

} // namespace prop_arm::ui::widgets

#include "prop_arm_gui/ui/widgets/chart_frame.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace prop_arm::ui::widgets {

ChartFrame::ChartFrame(QWidget* parent)
    : QFrame(parent),
      title_(new QLabel(this)),
      root_(new QVBoxLayout(this)) {

    setObjectName("chartFrame");
    setFrameShape(QFrame::NoFrame);

    // Important so the stylesheet background for QFrame actually paints.
    setAttribute(Qt::WA_StyledBackground, true);

    // Keep the frame fully controlled by stylesheet (no palette auto fill).
    setAutoFillBackground(false);

    // Frame padding must match chart cards.
    root_->setContentsMargins(16, 16, 16, 16);
    root_->setSpacing(12);

    // Title is optional; keep it hidden unless setTitle() is called.
    title_->setVisible(false);
    title_->setObjectName("chartFrameTitle");
    title_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

    // Put title on top always (even if empty/hidden).
    root_->addWidget(title_, 0);
}

void ChartFrame::setTitle(const QString& title) {
    title_->setText(title);
    title_->setVisible(!title.trimmed().isEmpty());
}

void ChartFrame::setChartWidget(QWidget* w) {
    if (!w) return;

    // If the widget is already inside another layout, detach it first.
    if (auto* oldParent = w->parentWidget(); oldParent && oldParent != this) {
        w->setParent(nullptr);
    }

    // Remove and delete previous chart widget safely (avoid layout stacking/ghosting).
    if (chart_) {
        root_->removeWidget(chart_);
        chart_->setParent(nullptr);
        chart_->deleteLater();
        chart_ = nullptr;
    }

    chart_ = w;
    chart_->setParent(this);
    chart_->setObjectName("chartWidget");
    chart_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Ensure the content doesn't draw its own opaque background unless it wants to.
    chart_->setAttribute(Qt::WA_StyledBackground, false);

    root_->addWidget(chart_, 1);
}

}  // namespace prop_arm::ui::widgets

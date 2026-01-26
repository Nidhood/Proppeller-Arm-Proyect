#pragma once

#include "prop_arm_gui/ui/style/theme.hpp"
#include <qchart.h>
#include <qlineseries.h>
#include <qvalueaxis.h>

namespace prop_arm::ui::style {

class ChartThemeApplier {
public:
    void apply(QChart& chart,
               QLegend* legend,
               QValueAxis& x,
               QValueAxis& y,
               QLineSeries* real,
               QLineSeries* sim,
               QLineSeries* ref,
               const ThemeSpec& spec,
               bool show_sim,
               bool show_ref,
               bool real_live,
               bool sim_live,
               bool ref_live,
               bool integer_y_axis) const;
};


} // namespace prop_arm::ui::style
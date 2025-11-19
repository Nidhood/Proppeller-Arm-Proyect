#pragma once

#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include <QCheckBox>
#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <map>
#include <vector>

class ExportPreviewDialog : public QDialog {
  Q_OBJECT

public:
  explicit ExportPreviewDialog(const std::vector<PropArmData> &data,
                               QWidget *parent = nullptr);
  ~ExportPreviewDialog() = default;

  std::vector<std::string> getSelectedColumns() const;
  bool wasAccepted() const { return accepted_; }

private slots:
  void onExportClicked();
  void onCancelClicked();
  void onSelectAllClicked();
  void onDeselectAllClicked();

private:
  void setupUI();
  void populateTable();
  QString formatNumber(double value, int decimals = 6) const;

  const std::vector<PropArmData> &data_;
  QTableWidget *table_widget_;
  std::map<std::string, QCheckBox *> column_checkboxes_;
  QPushButton *export_btn_;
  QPushButton *cancel_btn_;
  QPushButton *select_all_btn_;
  QPushButton *deselect_all_btn_;
  bool accepted_;

  std::vector<std::string> all_columns_ = {
      "Timestamp_ISO8601",     "Recording_Time_s",      "Arm_Angle_deg",
      "Ref_Angle_deg",         "Motor_Speed_rad_s",     "PWM_Input_us",
      "Duty_Cycle_percent",    "Tracking_Error_deg",    "Sim_Arm_Angle_deg",
      "Sim_Ref_Angle_deg",     "Sim_Motor_Speed_rad_s", "Sim_PWM_Input_us",
      "Sim_Duty_Cycle_percent"};
};
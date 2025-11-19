#include "prop_arm_gui/export_preview_dialog.hpp"
#include <QGroupBox>
#include <QHeaderView>
#include <QScrollArea>

ExportPreviewDialog::ExportPreviewDialog(const std::vector<PropArmData> &data,
                                         QWidget *parent)
    : QDialog(parent), data_(data), accepted_(false) {
  setupUI();
  populateTable();
}

void ExportPreviewDialog::setupUI() {
  setWindowTitle("Export Data Preview");
  setMinimumSize(1200, 600);

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setSpacing(10);

  QLabel *title_label =
      new QLabel("Select columns to export and preview data:");
  title_label->setStyleSheet(
      "font-size: 14px; font-weight: bold; color: #e4e4e7;");
  main_layout->addWidget(title_label);

  QGroupBox *columns_group = new QGroupBox("Column Selection");
  columns_group->setStyleSheet(
      "QGroupBox { color: #e4e4e7; font-weight: bold; }");
  QHBoxLayout *columns_layout = new QHBoxLayout(columns_group);

  QVBoxLayout *checkboxes_layout = new QVBoxLayout();

  for (const auto &col : all_columns_) {
    QCheckBox *checkbox = new QCheckBox(QString::fromStdString(col));
    checkbox->setChecked(true);
    checkbox->setStyleSheet("color: #e4e4e7;");
    column_checkboxes_[col] = checkbox;
    checkboxes_layout->addWidget(checkbox);

    connect(checkbox, &QCheckBox::toggled, this,
            &ExportPreviewDialog::populateTable);
  }

  columns_layout->addLayout(checkboxes_layout);

  QVBoxLayout *buttons_col_layout = new QVBoxLayout();
  select_all_btn_ = new QPushButton("Select All");
  deselect_all_btn_ = new QPushButton("Deselect All");

  select_all_btn_->setStyleSheet(
      "background-color: #424769; color: white; padding: 5px;");
  deselect_all_btn_->setStyleSheet(
      "background-color: #424769; color: white; padding: 5px;");

  connect(select_all_btn_, &QPushButton::clicked, this,
          &ExportPreviewDialog::onSelectAllClicked);
  connect(deselect_all_btn_, &QPushButton::clicked, this,
          &ExportPreviewDialog::onDeselectAllClicked);

  buttons_col_layout->addWidget(select_all_btn_);
  buttons_col_layout->addWidget(deselect_all_btn_);
  buttons_col_layout->addStretch();

  columns_layout->addLayout(buttons_col_layout);

  main_layout->addWidget(columns_group);

  QLabel *preview_label = new QLabel("Data Preview (first 5 rows):");
  preview_label->setStyleSheet(
      "font-size: 12px; font-weight: bold; color: #e4e4e7;");
  main_layout->addWidget(preview_label);

  table_widget_ = new QTableWidget(this);
  table_widget_->setStyleSheet(
      "QTableWidget { background-color: #1a1f3a; color: #e4e4e7; "
      "gridline-color: #424769; }"
      "QHeaderView::section { background-color: #2d3250; color: #e4e4e7; "
      "font-weight: bold; padding: 5px; }");
  table_widget_->setAlternatingRowColors(true);
  table_widget_->horizontalHeader()->setStretchLastSection(true);
  table_widget_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  main_layout->addWidget(table_widget_);

  QHBoxLayout *button_layout = new QHBoxLayout();
  button_layout->addStretch();

  cancel_btn_ = new QPushButton("Cancel");
  cancel_btn_->setStyleSheet("background-color: #f87171; color: white; "
                             "font-weight: bold; padding: 10px 30px;");
  connect(cancel_btn_, &QPushButton::clicked, this,
          &ExportPreviewDialog::onCancelClicked);

  export_btn_ = new QPushButton("Export");
  export_btn_->setStyleSheet("background-color: #4ade80; color: white; "
                             "font-weight: bold; padding: 10px 30px;");
  connect(export_btn_, &QPushButton::clicked, this,
          &ExportPreviewDialog::onExportClicked);

  button_layout->addWidget(cancel_btn_);
  button_layout->addWidget(export_btn_);

  main_layout->addLayout(button_layout);

  setStyleSheet("QDialog { background-color: #0a0e27; }");
}

void ExportPreviewDialog::populateTable() {
  std::vector<std::string> selected_columns = getSelectedColumns();

  table_widget_->clear();
  table_widget_->setColumnCount(selected_columns.size());
  table_widget_->setRowCount(std::min(size_t(5), data_.size()));

  QStringList headers;
  for (const auto &col : selected_columns) {
    headers << QString::fromStdString(col);
  }
  table_widget_->setHorizontalHeaderLabels(headers);

  if (data_.empty())
    return;

  double recording_start = data_.front().system_timestamp;

  for (size_t row = 0; row < std::min(size_t(5), data_.size()); ++row) {
    const auto &point = data_[row];
    double recording_time = point.system_timestamp - recording_start;
    QString timestamp = point.datetime.toString(Qt::ISODate);

    int col_idx = 0;
    for (const auto &col : selected_columns) {
      QString value;

      if (col == "Timestamp_ISO8601")
        value = timestamp;
      else if (col == "Recording_Time_s")
        value = formatNumber(recording_time);
      else if (col == "Arm_Angle_deg")
        value = formatNumber(point.arm_angle_deg);
      else if (col == "Ref_Angle_deg")
        value = formatNumber(point.ref_angle_deg);
      else if (col == "Motor_Speed_rad_s")
        value = formatNumber(point.motor_speed_rad_s);
      else if (col == "PWM_Input_us")
        value = formatNumber(point.pwm_input_us, 2);
      else if (col == "Duty_Cycle_percent")
        value = formatNumber(point.duty_cycle_percent, 4);
      else if (col == "Tracking_Error_deg")
        value = formatNumber(point.error);
      else if (col == "Sim_Arm_Angle_deg")
        value = formatNumber(point.sim_arm_angle_deg);
      else if (col == "Sim_Ref_Angle_deg")
        value = formatNumber(point.sim_ref_angle_deg);
      else if (col == "Sim_Motor_Speed_rad_s")
        value = formatNumber(point.sim_motor_speed_rad_s);
      else if (col == "Sim_PWM_Input_us")
        value = formatNumber(point.sim_pwm_input_us, 2);
      else if (col == "Sim_Duty_Cycle_percent")
        value = formatNumber(point.sim_duty_cycle_percent, 4);

      QTableWidgetItem *item = new QTableWidgetItem(value);
      item->setTextAlignment(Qt::AlignCenter);
      table_widget_->setItem(row, col_idx, item);
      col_idx++;
    }
  }

  table_widget_->resizeColumnsToContents();
}

QString ExportPreviewDialog::formatNumber(double value, int decimals) const {
  QString str = QString::number(value, 'f', decimals);
  str.replace(',', '.');
  return str;
}

std::vector<std::string> ExportPreviewDialog::getSelectedColumns() const {
  std::vector<std::string> selected;
  for (const auto &col : all_columns_) {
    if (column_checkboxes_.at(col)->isChecked()) {
      selected.push_back(col);
    }
  }
  return selected;
}

void ExportPreviewDialog::onExportClicked() {
  if (getSelectedColumns().empty()) {
    return;
  }
  accepted_ = true;
  accept();
}

void ExportPreviewDialog::onCancelClicked() {
  accepted_ = false;
  reject();
}

void ExportPreviewDialog::onSelectAllClicked() {
  for (auto &pair : column_checkboxes_) {
    pair.second->setChecked(true);
  }
}

void ExportPreviewDialog::onDeselectAllClicked() {
  for (auto &pair : column_checkboxes_) {
    pair.second->setChecked(false);
  }
}
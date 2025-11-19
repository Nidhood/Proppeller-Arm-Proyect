#include "prop_arm_gui/data_exporter.hpp"
#include <QDir>
#include <QFileInfo>
#include <algorithm>

QString DataExporter::formatNumber(double value, int decimals) const {
  QString str = QString::number(value, 'f', decimals);
  str.replace(',', '.');
  return str;
}

bool DataExporter::exportToCSV(
    const QString &filename, const std::vector<PropArmData> &data,
    double simulation_start_time,
    const std::vector<std::string> &selected_columns) {
  Q_UNUSED(simulation_start_time);

  if (data.empty()) {
    last_error_ = "No data to export";
    return false;
  }

  if (selected_columns.empty()) {
    last_error_ = "No columns selected for export";
    return false;
  }

  QFileInfo file_info(filename);
  QDir dir = file_info.absoluteDir();
  if (!dir.exists()) {
    if (!dir.mkpath(".")) {
      last_error_ = "Failed to create directory: " + dir.absolutePath();
      return false;
    }
  }

  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    last_error_ = "Failed to open file: " + filename;
    return false;
  }

  QTextStream out(&file);

  out << "# PropArm Data Recording\n";
  out << "# Export Time: " << QDateTime::currentDateTime().toString(Qt::ISODate)
      << "\n";
  out << "# Number of Points: " << data.size() << "\n";

  if (!data.empty()) {
    double recording_duration =
        data.back().system_timestamp - data.front().system_timestamp;
    out << "# Recording Duration: " << formatNumber(recording_duration, 3)
        << " seconds\n";
    out << "# Start Time: " << data.front().datetime.toString(Qt::ISODate)
        << "\n";
    out << "# End Time: " << data.back().datetime.toString(Qt::ISODate) << "\n";
  }
  out << "#\n";

  for (size_t i = 0; i < selected_columns.size(); ++i) {
    out << QString::fromStdString(selected_columns[i]);
    if (i < selected_columns.size() - 1)
      out << ",";
  }
  out << "\n";

  double recording_start = data.front().system_timestamp;

  for (const auto &point : data) {
    double recording_time = point.system_timestamp - recording_start;
    QString timestamp = point.datetime.toString(Qt::ISODate);

    for (size_t i = 0; i < selected_columns.size(); ++i) {
      const std::string &col = selected_columns[i];

      if (col == "Timestamp_ISO8601")
        out << timestamp;
      else if (col == "Recording_Time_s")
        out << formatNumber(recording_time);
      else if (col == "Arm_Angle_deg")
        out << formatNumber(point.arm_angle_deg);
      else if (col == "Ref_Angle_deg")
        out << formatNumber(point.ref_angle_deg);
      else if (col == "Motor_Speed_rad_s")
        out << formatNumber(point.motor_speed_rad_s);
      else if (col == "PWM_Input_us")
        out << formatNumber(point.pwm_input_us, 2);
      else if (col == "Duty_Cycle_percent")
        out << formatNumber(point.duty_cycle_percent, 4);
      else if (col == "Tracking_Error_deg")
        out << formatNumber(point.error);
      else if (col == "Sim_Arm_Angle_deg")
        out << formatNumber(point.sim_arm_angle_deg);
      else if (col == "Sim_Ref_Angle_deg")
        out << formatNumber(point.sim_ref_angle_deg);
      else if (col == "Sim_Motor_Speed_rad_s")
        out << formatNumber(point.sim_motor_speed_rad_s);
      else if (col == "Sim_PWM_Input_us")
        out << formatNumber(point.sim_pwm_input_us, 2);
      else if (col == "Sim_Duty_Cycle_percent")
        out << formatNumber(point.sim_duty_cycle_percent, 4);

      if (i < selected_columns.size() - 1)
        out << ",";
    }
    out << "\n";
  }

  file.close();

  last_error_.clear();
  return true;
}

bool DataExporter::exportLastSeconds(
    const QString &filename, const std::deque<PropArmData> &data,
    double duration_seconds, double simulation_start_time,
    const std::vector<std::string> &selected_columns) {
  if (data.empty()) {
    last_error_ = "No data to export";
    return false;
  }

  double latest_time = data.back().system_timestamp;
  double cutoff_time = latest_time - duration_seconds;

  std::vector<PropArmData> filtered_data;
  for (const auto &point : data) {
    if (point.system_timestamp >= cutoff_time) {
      filtered_data.push_back(point);
    }
  }

  if (filtered_data.empty()) {
    last_error_ = "No data in the specified time window";
    return false;
  }

  return exportToCSV(filename, filtered_data, simulation_start_time,
                     selected_columns);
}

QString DataExporter::formatTimestamp(const rclcpp::Time &ros_time,
                                      double simulation_start_time) const {
  Q_UNUSED(simulation_start_time);
  Q_UNUSED(ros_time);

  return QDateTime::currentDateTime().toString(Qt::ISODate);
}

double DataExporter::getRelativeTime(const rclcpp::Time &ros_time,
                                     double simulation_start_time) const {
  Q_UNUSED(ros_time);
  Q_UNUSED(simulation_start_time);

  return 0.0;
}
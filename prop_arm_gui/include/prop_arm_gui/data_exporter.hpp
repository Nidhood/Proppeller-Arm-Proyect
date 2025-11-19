#pragma once

#include "prop_arm_gui/prop_arm_gui_node.hpp"
#include <QDateTime>
#include <QFile>
#include <QString>
#include <QTextStream>
#include <deque>
#include <vector>

class DataExporter {
public:
  DataExporter() = default;
  ~DataExporter() = default;

  bool exportToCSV(const QString &filename,
                   const std::vector<PropArmData> &data,
                   double simulation_start_time,
                   const std::vector<std::string> &selected_columns);

  bool exportLastSeconds(const QString &filename,
                         const std::deque<PropArmData> &data,
                         double duration_seconds, double simulation_start_time,
                         const std::vector<std::string> &selected_columns);

  QString getLastError() const { return last_error_; }

private:
  QString last_error_;

  QString formatTimestamp(const rclcpp::Time &ros_time,
                          double simulation_start_time) const;

  double getRelativeTime(const rclcpp::Time &ros_time,
                         double simulation_start_time) const;

  QString formatNumber(double value, int decimals = 6) const;
};
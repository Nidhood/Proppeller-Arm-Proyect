#pragma once

#include <QString>
#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <vector>
#include <deque>
#include "prop_arm_gui/prop_arm_gui_node.hpp"

class DataExporter
{
public:
    DataExporter() = default;
    ~DataExporter() = default;

    // Export data to CSV
    bool exportToCSV(const QString &filename, 
                     const std::vector<PropArmData> &data,
                     double simulation_start_time);

    // Export data with time window (last N seconds)
    bool exportLastSeconds(const QString &filename,
                          const std::deque<PropArmData> &data,
                          double duration_seconds,
                          double simulation_start_time);

    // Get last error message
    QString getLastError() const { return last_error_; }

private:
    QString last_error_;

    // Helper function to format timestamp
    QString formatTimestamp(const rclcpp::Time &ros_time, double simulation_start_time) const;
    
    // Helper function to calculate relative time
    double getRelativeTime(const rclcpp::Time &ros_time, double simulation_start_time) const;
};
#include "prop_arm_gui/data_exporter.hpp"
#include <QFileInfo>
#include <QDir>
#include <algorithm>

bool DataExporter::exportToCSV(const QString &filename,
                               const std::vector<PropArmData> &data,
                               double simulation_start_time)
{
    Q_UNUSED(simulation_start_time);

    if (data.empty())
    {
        last_error_ = "No data to export";
        return false;
    }

    QFileInfo file_info(filename);
    QDir dir = file_info.absoluteDir();
    if (!dir.exists())
    {
        if (!dir.mkpath("."))
        {
            last_error_ = "Failed to create directory: " + dir.absolutePath();
            return false;
        }
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        last_error_ = "Failed to open file: " + filename;
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberPrecision(6);

    out << "# PropArm Data Recording\n";
    out << "# Export Time: " << QDateTime::currentDateTime().toString(Qt::ISODate) << "\n";
    out << "# Number of Points: " << data.size() << "\n";

    if (!data.empty())
    {
        double recording_duration = data.back().system_timestamp - data.front().system_timestamp;
        out << "# Recording Duration: " << QString::number(recording_duration, 'f', 3) << " seconds\n";
        out << "# Start Time: " << data.front().datetime.toString(Qt::ISODate) << "\n";
        out << "# End Time: " << data.back().datetime.toString(Qt::ISODate) << "\n";
    }
    out << "#\n";

    out << "Timestamp_ISO8601,";
    out << "Recording_Time_s,";
    out << "Arm_Angle_deg,";
    out << "Ref_Angle_deg,";
    out << "Motor_Speed_rad_s,";
    out << "PWM_Input_us,";
    out << "Duty_Cycle_percent,";
    out << "Tracking_Error_deg,";
    out << "Sim_Arm_Angle_deg,";
    out << "Sim_Ref_Angle_deg,";
    out << "Sim_Motor_Speed_rad_s,";
    out << "Sim_PWM_Input_us,";
    out << "Sim_Duty_Cycle_percent\n";

    double recording_start = data.front().system_timestamp;

    for (const auto &point : data)
    {
        double recording_time = point.system_timestamp - recording_start;
        QString timestamp = point.datetime.toString(Qt::ISODate);

        out << timestamp << ",";
        out << QString::number(recording_time, 'f', 6) << ",";
        out << QString::number(point.arm_angle_deg, 'f', 6) << ",";
        out << QString::number(point.ref_angle_deg, 'f', 6) << ",";
        out << QString::number(point.motor_speed_rad_s, 'f', 6) << ",";
        out << QString::number(point.pwm_input_us, 'f', 2) << ",";
        out << QString::number(point.duty_cycle_percent, 'f', 4) << ",";
        out << QString::number(point.error, 'f', 6) << ",";
        out << QString::number(point.sim_arm_angle_deg, 'f', 6) << ",";
        out << QString::number(point.sim_ref_angle_deg, 'f', 6) << ",";
        out << QString::number(point.sim_motor_speed_rad_s, 'f', 6) << ",";
        out << QString::number(point.sim_pwm_input_us, 'f', 2) << ",";
        out << QString::number(point.sim_duty_cycle_percent, 'f', 4) << "\n";
    }

    file.close();

    last_error_.clear();
    return true;
}

bool DataExporter::exportLastSeconds(const QString &filename,
                                     const std::deque<PropArmData> &data,
                                     double duration_seconds,
                                     double simulation_start_time)
{
    if (data.empty())
    {
        last_error_ = "No data to export";
        return false;
    }

    double latest_time = data.back().system_timestamp;
    double cutoff_time = latest_time - duration_seconds;

    std::vector<PropArmData> filtered_data;
    for (const auto &point : data)
    {
        if (point.system_timestamp >= cutoff_time)
        {
            filtered_data.push_back(point);
        }
    }

    if (filtered_data.empty())
    {
        last_error_ = "No data in the specified time window";
        return false;
    }

    return exportToCSV(filename, filtered_data, simulation_start_time);
}

QString DataExporter::formatTimestamp(const rclcpp::Time &ros_time,
                                      double simulation_start_time) const
{
    Q_UNUSED(simulation_start_time);
    Q_UNUSED(ros_time);

    return QDateTime::currentDateTime().toString(Qt::ISODate);
}

double DataExporter::getRelativeTime(const rclcpp::Time &ros_time,
                                     double simulation_start_time) const
{
    Q_UNUSED(ros_time);
    Q_UNUSED(simulation_start_time);

    return 0.0;
}
#include "prop_arm_gui/data_exporter.hpp"
#include <QFileInfo>
#include <QDir>
#include <algorithm>

bool DataExporter::exportToCSV(const QString &filename, 
                                const std::vector<PropArmData> &data,
                                double simulation_start_time)
{
    // Silenciar warning de parámetro no usado
    Q_UNUSED(simulation_start_time);
    
    if (data.empty())
    {
        last_error_ = "No data to export";
        return false;
    }

    // Ensure directory exists
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

    // Write header with metadata
    out << "# PropArm Data Recording\n";
    out << "# Export Time: " << QDateTime::currentDateTime().toString(Qt::ISODate) << "\n";
    out << "# Number of Points: " << data.size() << "\n";
    
    if (!data.empty())
    {
        // FIXED: Usar system_timestamp en lugar de timestamp ROS
        double recording_duration = data.back().system_timestamp - data.front().system_timestamp;
        out << "# Recording Duration: " << QString::number(recording_duration, 'f', 3) << " seconds\n";
        out << "# Start Time: " << data.front().datetime.toString(Qt::ISODate) << "\n";
        out << "# End Time: " << data.back().datetime.toString(Qt::ISODate) << "\n";
    }
    out << "#\n";

    // Write CSV header
    out << "Timestamp_ISO8601,";
    out << "Recording_Time_s,";
    out << "Arm_Angle_deg,";
    out << "Motor_Speed_rad_s,";
    out << "PWM_Input_us,";
    out << "Duty_Cycle_percent,";
    out << "Motor_Command,";
    out << "Control_Error_deg,";
    out << "Target_Angle_deg\n";

    // FIXED: Calcular tiempo de inicio de la grabación (primer punto)
    double recording_start = data.front().system_timestamp;

    // Write data
    for (const auto &point : data)
    {
        // FIXED: Calcular tiempo relativo usando system_timestamp
        double recording_time = point.system_timestamp - recording_start;
        QString timestamp = point.datetime.toString(Qt::ISODate);

        out << timestamp << ",";
        out << QString::number(recording_time, 'f', 6) << ",";
        out << QString::number(point.arm_angle_deg, 'f', 6) << ",";
        out << QString::number(point.motor_speed_rad_s, 'f', 6) << ",";
        out << QString::number(point.pwm_input_us, 'f', 2) << ",";
        out << QString::number(point.duty_cycle_percent, 'f', 4) << ",";
        out << QString::number(point.motor_command, 'f', 6) << ",";
        out << QString::number(point.error, 'f', 6) << ",";
        out << QString::number(point.target_angle, 'f', 6) << "\n";
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

    // FIXED: Usar system_timestamp en lugar de ROS time
    double latest_time = data.back().system_timestamp;
    double cutoff_time = latest_time - duration_seconds;

    // Filter data
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
    // DEPRECATED: Esta función ya no se usa, pero se mantiene por compatibilidad
    Q_UNUSED(simulation_start_time);
    Q_UNUSED(ros_time);
    
    return QDateTime::currentDateTime().toString(Qt::ISODate);
}

double DataExporter::getRelativeTime(const rclcpp::Time &ros_time, 
                                     double simulation_start_time) const
{
    // DEPRECATED: Esta función ya no se usa, pero se mantiene por compatibilidad
    Q_UNUSED(ros_time);
    Q_UNUSED(simulation_start_time);
    
    return 0.0;
}
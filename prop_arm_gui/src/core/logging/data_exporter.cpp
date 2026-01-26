#include "prop_arm_gui/core/logging/data_exporter.hpp"
#include "prop_arm_gui/core/models/csv_export_options.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace prop_arm::core::logging {

static std::vector<std::string> defaultColumns() {
    return {
        "Domain",
        "t_rel_s",
        "arm_angle_rad",
        "motor_speed_rad_s",
        "pwm_us",
        "ref_angle_rad"
    };
}

bool DataExporter::exportToCsv(const std::string& filename,
                               const std::vector<prop_arm::models::TelemetrySample>& samples,
                               const prop_arm::models::CsvExportOptions& opt) {
    last_error_.clear();

    if (samples.empty()) {
        last_error_ = "No data to export (samples is empty).";
        return false;
    }

    // Ensure directory exists
    try {
        std::filesystem::path p(filename);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to create output directory: ") + e.what();
        return false;
    }

    std::ofstream out(filename, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        last_error_ = "Failed to open output file: " + filename;
        return false;
    }

    const auto columns = opt.columns.empty() ? defaultColumns() : opt.columns;

    std::vector<prop_arm::models::TelemetrySample> data = samples;
    std::sort(data.begin(), data.end(),
    [](const auto& a, const auto& b) {
        return a.t < b.t;
    });

    const auto& first = data.front();

    if (opt.include_header_comments) {
        out << "# PropArm telemetry export\n";
        out << "# Points: " << data.size() << "\n";
        out << "# Columns: " << join_(columns, ',') << "\n";
        out << "#\n";
    }

    for (std::size_t i = 0; i < columns.size(); ++i) {
        out << columns[i];
        if (i + 1 < columns.size()) out << ",";
    }
    out << "\n";
    for (const auto& s : data) {
        for (std::size_t i = 0; i < columns.size(); ++i) {
            const auto& col = columns[i];

            if (col == "Domain") {
                out << domainToString_(s.domain);
            } else if (col == "t_rel_s") {
                out << formatFloat_(relTimeSeconds_(s, first), opt.decimals);
            } else if (col == "arm_angle_rad") {
                out << formatFloat_(s.arm_angle_rad, opt.decimals);
            } else if (col == "motor_speed_rad_s") {
                out << formatFloat_(s.motor_speed_rad_s, opt.decimals);
            } else if (col == "pwm_us") {
                out << s.pwm_us;
            } else if (col == "ref_angle_rad") {
                out << formatFloat_(s.ref_angle_rad, opt.decimals);
            } else if (col == "valid") {
                out << (s.valid ? "1" : "0");
            } else {
                // Unknown column: keep CSV shape but mark the error.
                out << "";
                last_error_ = "Unknown column name in export request: " + col;
            }

            if (i + 1 < columns.size()) out << ",";
        }
        out << "\n";
    }

    out.close();
    return last_error_.empty();
}

std::string DataExporter::domainToString_(prop_arm::models::Domain d) {
    return (d == prop_arm::models::Domain::Real) ? "real" : "sim";
}

double DataExporter::relTimeSeconds_(const prop_arm::models::TelemetrySample& s,
                                     const prop_arm::models::TelemetrySample& first) {
    using namespace std::chrono;
    return duration_cast<duration<double>>(s.t - first.t).count();
}

std::string DataExporter::formatFloat_(double v, int decimals) {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(decimals) << v;
    return ss.str();
}

std::string DataExporter::join_(const std::vector<std::string>& cols, const char sep) {
    std::ostringstream ss;
    for (std::size_t i = 0; i < cols.size(); ++i) {
        ss << cols[i];
        if (i + 1 < cols.size()) ss << sep;
    }
    return ss.str();
}

} // namespace prop_arm::core::logging
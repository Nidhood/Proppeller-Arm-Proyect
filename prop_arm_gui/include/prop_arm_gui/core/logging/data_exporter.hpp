#pragma once

#include <string>
#include <vector>

#include "prop_arm_gui/core/models/telemetry_sample.hpp"
#include "prop_arm_gui/core/models/csv_export_options.hpp"


namespace prop_arm::core::logging {

class DataExporter final {

public:
    DataExporter() = default;
    bool exportToCsv(const std::string& filename,
                     const std::vector<prop_arm::models::TelemetrySample>& samples,
                     const prop_arm::models::CsvExportOptions& opt = {});
    const std::string& lastError() const {
        return last_error_;
    }

private:
    std::string last_error_;
    static std::string domainToString_(prop_arm::models::Domain d);
    static double relTimeSeconds_(const prop_arm::models::TelemetrySample& s,
                                  const prop_arm::models::TelemetrySample& first);
    static std::string formatFloat_(double v, int decimals);
    static std::string join_(const std::vector<std::string>& cols, const char sep);
};

} // namespace prop_arm_gui::core::domain
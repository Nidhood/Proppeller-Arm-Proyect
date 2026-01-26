#pragma once

#include <string>
#include <vector>

namespace prop_arm::models {

struct CsvExportOptions {

    std::vector<std::string> columns;
    int decimals = 6;
    bool include_header_comments = true;
};

} // namespace prop_arm::models



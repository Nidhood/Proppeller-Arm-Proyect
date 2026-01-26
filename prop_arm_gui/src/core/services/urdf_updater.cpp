#include "prop_arm_gui/core/services/urdf_updater.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <chrono>
#include <iomanip>

namespace prop_arm::services {

UrdfUpdater::UrdfUpdater(const std::filesystem::path& workspace_path)
    : workspace_path_(workspace_path)
{
    control_xacro_path_ = workspace_path_ / "src" / "prop_arm_gazebo_control" /
                          "urdf" / "prop_arm_control.xacro";

    urdf_xacro_path_ = workspace_path_ / "src" / "prop_arm_description" /
                       "urdf" / "prop_arm.urdf.xacro";

    backup_dir_ = workspace_path_ / "calibration_backups";

    std::filesystem::create_directories(backup_dir_);
}

bool UrdfUpdater::updateMotorParameters(
    const models::MotorVelocityParams& params)
{
    last_error_.clear();

    auto content = readFile_(control_xacro_path_);
    if (!content) {
        last_error_ = "Failed to read control xacro file";
        return false;
    }

    std::string updated = *content;

    updated = replaceParameter_(updated, "Kw", params.Kw);
    updated = replaceParameter_(updated, "tau_w", params.tau_w);
    updated = replaceParameter_(updated, "L_w", params.L_w);
    updated = replaceParameter_(updated, "motor_cmd_scale", params.motor_cmd_scale);

    if (!writeFile_(control_xacro_path_, updated)) {
        last_error_ = "Failed to write updated control xacro file";
        return false;
    }

    return true;
}

bool UrdfUpdater::updatePhysicsParameters(
    const models::ArmPhysicsParams& params)
{
    last_error_.clear();

    auto content = readFile_(urdf_xacro_path_);
    if (!content) {
        last_error_ = "Failed to read URDF xacro file";
        return false;
    }

    std::string updated = *content;

    // Update arm_link inertial properties
    updated = replaceParameterInTag_(updated, "arm_link", "mass", params.mass);
    updated = replaceParameterInTag_(updated, "arm_link", "iyy", params.inertia_yy);

    // Update arm_link_joint dynamics
    updated = replaceParameterInTag_(updated, "arm_link_joint", "damping", params.damping);
    updated = replaceParameterInTag_(updated, "arm_link_joint", "friction", params.friction);

    if (!writeFile_(urdf_xacro_path_, updated)) {
        last_error_ = "Failed to write updated URDF xacro file";
        return false;
    }

    return true;
}

bool UrdfUpdater::updateAllParameters(
    const models::MotorVelocityParams& motor_params,
    const models::ArmPhysicsParams& physics_params)
{
    if (!createBackup()) {
        last_error_ = "Failed to create backup before update";
        return false;
    }

    if (!updateMotorParameters(motor_params)) {
        restoreFromBackup();
        return false;
    }

    if (!updatePhysicsParameters(physics_params)) {
        restoreFromBackup();
        return false;
    }

    return true;
}

bool UrdfUpdater::createBackup()
{
    const auto now = std::chrono::system_clock::now();
    const auto time_t = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << "backup_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

    const auto backup_subdir = backup_dir_ / oss.str();
    std::filesystem::create_directories(backup_subdir);

    try {
        std::filesystem::copy_file(
            control_xacro_path_,
            backup_subdir / "prop_arm_control.xacro",
            std::filesystem::copy_options::overwrite_existing);

        std::filesystem::copy_file(
            urdf_xacro_path_,
            backup_subdir / "prop_arm.urdf.xacro",
            std::filesystem::copy_options::overwrite_existing);

        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        last_error_ = std::string("Backup failed: ") + e.what();
        return false;
    }
}

bool UrdfUpdater::restoreFromBackup()
{
    // Find most recent backup
    std::filesystem::path latest_backup;
    std::filesystem::file_time_type latest_time;

    try {
        for (const auto& entry : std::filesystem::directory_iterator(backup_dir_)) {
            if (!entry.is_directory()) continue;

            const auto time = std::filesystem::last_write_time(entry);
            if (latest_backup.empty() || time > latest_time) {
                latest_backup = entry.path();
                latest_time = time;
            }
        }

        if (latest_backup.empty()) {
            last_error_ = "No backup found";
            return false;
        }

        std::filesystem::copy_file(
            latest_backup / "prop_arm_control.xacro",
            control_xacro_path_,
            std::filesystem::copy_options::overwrite_existing);

        std::filesystem::copy_file(
            latest_backup / "prop_arm.urdf.xacro",
            urdf_xacro_path_,
            std::filesystem::copy_options::overwrite_existing);

        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        last_error_ = std::string("Restore failed: ") + e.what();
        return false;
    }
}

bool UrdfUpdater::validateUrdfFile(const std::filesystem::path& path) const
{
    if (!std::filesystem::exists(path)) {
        return false;
    }

    const auto content = readFile_(path);
    if (!content) {
        return false;
    }

    // Basic validation: check for required XML structure
    return content->find("<?xml") != std::string::npos &&
           content->find("<robot") != std::string::npos;
}

std::optional<std::string> UrdfUpdater::getLastError() const noexcept
{
    if (last_error_.empty()) {
        return std::nullopt;
    }
    return last_error_;
}

std::string UrdfUpdater::replaceParameter_(
    const std::string& content,
    const std::string& param_name,
    double value) const
{
    std::ostringstream value_str;
    value_str << std::fixed << std::setprecision(6) << value;

    // Match: <param name="param_name">old_value</param>
    std::regex pattern(
        "<param\\s+name=\"" + param_name + "\"[^>]*>([^<]*)</param>",
        std::regex::icase);

    const std::string replacement =
        "<param name=\"" + param_name + "\">" + value_str.str() + "</param>";

    return std::regex_replace(content, pattern, replacement);
}

std::string UrdfUpdater::replaceParameterInTag_(
    const std::string& content,
    const std::string& tag_name,
    const std::string& param_name,
    double value) const
{
    std::ostringstream value_str;
    value_str << std::fixed << std::setprecision(6) << value;

    // Find the tag section
    const std::string start_tag = "<" + tag_name;
    const std::string end_tag = "</" + tag_name + ">";

    std::size_t tag_start = content.find(start_tag);
    if (tag_start == std::string::npos) {
        return content;
    }

    std::size_t tag_end = content.find(end_tag, tag_start);
    if (tag_end == std::string::npos) {
        return content;
    }

    const std::string tag_content = content.substr(
                                        tag_start, tag_end - tag_start + end_tag.length());

    std::string updated_tag = tag_content;

    // Handle different XML attribute formats
    if (param_name == "mass") {
        std::regex pattern("<mass\\s+value=\"([^\"]+)\"");
        updated_tag = std::regex_replace(
                          updated_tag, pattern,
                          "<mass value=\"" + value_str.str() + "\"");
    } else if (param_name == "iyy") {
        std::regex pattern("iyy=\"([^\"]+)\"");
        updated_tag = std::regex_replace(
                          updated_tag, pattern,
                          "iyy=\"" + value_str.str() + "\"");
    } else if (param_name == "damping") {
        std::regex pattern("damping=\"([^\"]+)\"");
        updated_tag = std::regex_replace(
                          updated_tag, pattern,
                          "damping=\"" + value_str.str() + "\"");
    } else if (param_name == "friction") {
        std::regex pattern("friction=\"([^\"]+)\"");
        updated_tag = std::regex_replace(
                          updated_tag, pattern,
                          "friction=\"" + value_str.str() + "\"");
    }

    std::string result = content;
    result.replace(tag_start, tag_content.length(), updated_tag);

    return result;
}

bool UrdfUpdater::writeFile_(
    const std::filesystem::path& path,
    const std::string& content)
{
    try {
        std::ofstream file(path);
        if (!file.is_open()) {
            return false;
        }

        file << content;
        file.close();

        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::optional<std::string> UrdfUpdater::readFile_(
    const std::filesystem::path& path) const
{
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            return std::nullopt;
        }

        std::ostringstream oss;
        oss << file.rdbuf();

        return oss.str();
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

} // namespace prop_arm::services
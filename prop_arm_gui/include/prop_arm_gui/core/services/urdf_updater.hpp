#pragma once

#include "prop_arm_gui/core/models/calibration_types.hpp"
#include <optional>
#include <string>
#include <filesystem>

namespace prop_arm::services {

class UrdfUpdater final {
public:
    explicit UrdfUpdater(const std::filesystem::path& workspace_path);

    // Update URDF files with new parameters
    bool updateMotorParameters(const models::MotorVelocityParams& params);
    bool updatePhysicsParameters(const models::ArmPhysicsParams& params);
    bool updateAllParameters(
        const models::MotorVelocityParams& motor_params,
        const models::ArmPhysicsParams& physics_params);

    // Backup and restore
    bool createBackup();
    bool restoreFromBackup();

    // Validation
    [[nodiscard]] bool validateUrdfFile(const std::filesystem::path& path) const;
    [[nodiscard]] std::optional<std::string> getLastError() const noexcept;

private:
    std::string generateControlXacro_(
        const models::MotorVelocityParams& motor_params) const;

    std::string generateUrdfXacro_(
        const models::ArmPhysicsParams& physics_params) const;

    bool writeFile_(const std::filesystem::path& path, const std::string& content);
    std::optional<std::string> readFile_(const std::filesystem::path& path) const;

    std::string replaceParameter_(
        const std::string& content,
        const std::string& param_name,
        double value) const;

    std::string replaceParameterInTag_(
        const std::string& content,
        const std::string& tag_name,
        const std::string& param_name,
        double value) const;

private:
    std::filesystem::path workspace_path_;
    std::filesystem::path control_xacro_path_;
    std::filesystem::path urdf_xacro_path_;
    std::filesystem::path backup_dir_;

    mutable std::string last_error_;
};

} // namespace prop_arm::services
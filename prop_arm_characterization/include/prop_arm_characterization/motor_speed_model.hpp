#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>   // IWYU pragma: keep
#include <vector>

namespace prop_arm_characterization {

class MotorSpeedModel {
public:
    MotorSpeedModel() = default;

    MotorSpeedModel(double Ts,
                    double L_w,
                    double c1,
                    double c2,
                    double d1,
                    double d2,
                    std::uint16_t pwm_min_us,
                    std::uint16_t pwm_max_us,
                    std::uint16_t pwm_spin_min_us,
                    double motor_cmd_scale,
                    double w0 = 0.0,
                    double prop_radius_m = 0.0) noexcept;

    virtual ~MotorSpeedModel() = default;

    // Update model with a new PWM command (microseconds). Returns w (rad/s).
    double update(std::uint16_t pwm_us) noexcept;

    [[nodiscard]] double getSpeedRadSec() const noexcept {
        return w_;
    }

    // Reset state (w[k-1], w[k-2]) to w0 and clear input history.
    void reset(double w0 = 0.0) noexcept;

    bool operator==(const MotorSpeedModel& other) const noexcept;

private:
    [[nodiscard]] std::uint16_t clampPwm_(std::uint16_t pwm_us) const noexcept;

    // Configuration
    double Ts_{0.01};
    double L_w_{0.0};
    int delay_steps_{0};

    double c1_{0.0};
    double c2_{0.0};
    double d1_{0.0};
    double d2_{0.0};

    std::uint16_t pwm_min_us_{1000};
    std::uint16_t pwm_max_us_{2000};
    std::uint16_t pwm_spin_min_us_{1050};

    double motor_cmd_scale_{1.0};

    // Optional (kept for compatibility with your YAML; not used inside ARX update)
    double prop_radius_m_{0.0};

    // State: w[k], w[k-1], w[k-2]
    double w_{0.0};
    double w_k_1_{0.0};
    double w_k_2_{0.0};

    // Input ring buffer holding u_eff history (size = delay_steps + 3)
    std::vector<double> u_ring_{};
    std::size_t write_idx_{0};
};

}  // namespace prop_arm_characterization

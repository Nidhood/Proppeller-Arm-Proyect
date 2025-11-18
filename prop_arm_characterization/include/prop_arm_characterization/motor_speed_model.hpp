#pragma once
#include <cmath>
#include <cstdint> // IWYU pragma: keep

namespace prop_arm_characterization
{
    class MotorSpeedModel
    {
    protected:
        double a_{0.0};
        double b_{0.0};
        std::uint16_t u_ref_{0}; // Reference PWM [us]
        double w_{0.0};          // Current speed [rad/s]
        double prop_radius_m_{0.0};

    public:
        MotorSpeedModel() = default;
        MotorSpeedModel(double kw,
                        double taw_w,
                        double Ts,
                        std::uint16_t pwm_ref_us,
                        double w0) noexcept;
        virtual ~MotorSpeedModel() = default;

        // Update model with PWM (UInt16) and return speed [rad/s]
        double update(std::uint16_t pwm_us) noexcept;

        double getSpeedRadSec() const noexcept;
        void reset() noexcept;

        bool operator==(const MotorSpeedModel &other) const noexcept;
    };
}

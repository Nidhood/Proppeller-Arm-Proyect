#pragma once
#include <cmath>
#include <cstdint> // IWYU pragma: keep

namespace prop_arm_gazebo_control
{
    class MotorSpeedModel
    {
    protected:
        double a_{0.0};
        double b_{0.0};
        int u_ref_{0};
        double w_{0.0};
        double prop_radius_m_{0.0};

    public:
        MotorSpeedModel() = default;
        MotorSpeedModel(double kw, double taw_w, double Ts, int pwm_ref_us, double w0) noexcept;
        virtual ~MotorSpeedModel() = default;
        double update(int pwm_us) noexcept;
        double getSpeedRadSec() const noexcept;
        void reset() noexcept;
        bool operator==(const MotorSpeedModel &other) const noexcept;
    };
}

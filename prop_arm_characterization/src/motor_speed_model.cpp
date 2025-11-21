#include "prop_arm_characterization/motor_speed_model.hpp"

namespace prop_arm_characterization
{

MotorSpeedModel::MotorSpeedModel(double kw,
                                 double tau_w,
                                 double Ts,
                                 std::uint16_t pwm_ref_us,
                                 double w0) noexcept
    : u_ref_(pwm_ref_us), w_(w0)
{
    if (tau_w > 0.0 && Ts > 0.0)
    {
        const double ratio = -Ts / tau_w;
        a_ = std::exp(ratio);
        b_ = kw * (1.0 - a_);
    }
    else
    {
        a_ = 0.0;
        b_ = 0.0;
    }
}

double MotorSpeedModel::update(std::uint16_t pwm_us) noexcept
{
    const double u_delta =
        static_cast<double>(pwm_us) - static_cast<double>(u_ref_);

    w_ = a_ * w_ + b_ * u_delta;
    return w_;
}

double MotorSpeedModel::getSpeedRadSec() const noexcept
{
    return w_;
}

void MotorSpeedModel::reset() noexcept
{
    w_ = 0.0;
}

bool MotorSpeedModel::operator==(const MotorSpeedModel &other) const noexcept
{
    return (a_ == other.a_) &&
           (b_ == other.b_) &&
           (u_ref_ == other.u_ref_) &&
           (w_ == other.w_);
}

} // namespace prop_arm_characterization

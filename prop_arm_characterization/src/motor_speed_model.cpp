#include "prop_arm_characterization/motor_speed_model.hpp"

namespace prop_arm_characterization
{

// Constructor with parameters.
MotorSpeedModel::MotorSpeedModel(double kw,
                                 double taw_w,
                                 double Ts,
                                 std::uint16_t pwm_ref_us,
                                 double w0) noexcept
    : u_ref_(pwm_ref_us), w_(w0)
{
    a_ = std::exp(-Ts / taw_w);
    b_ = kw * (1.0 - a_);
}

// Simple first-order motor speed model implementation PWM [us] to speed [rad/s].
double MotorSpeedModel::update(std::uint16_t pwm_us) noexcept
{
    const double du =
        static_cast<double>(pwm_us) - static_cast<double>(u_ref_);
    w_ = a_ * w_ + b_ * du;
    return w_;
}

// Get current speed [rad/s].
double MotorSpeedModel::getSpeedRadSec() const noexcept
{
    return w_;
}

// Reset model state.
void MotorSpeedModel::reset() noexcept
{
    w_ = 0.0;
}

// Equality operator.
bool MotorSpeedModel::operator==(const MotorSpeedModel &other) const noexcept
{
    return (a_ == other.a_) &&
           (b_ == other.b_) &&
           (u_ref_ == other.u_ref_) &&
           (w_ == other.w_);
}

} // namespace prop_arm_characterization

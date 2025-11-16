#include "prop_arm_gazebo_control/motor_speed_model.hpp"

// Constructor with parameters:
prop_arm_gazebo_control::MotorSpeedModel::MotorSpeedModel(double kw, double taw_w, double Ts, int pwm_ref_us, double w0) noexcept
    : u_ref_(pwm_ref_us), w_(w0)
{
    a_ = std::exp(-Ts / taw_w);
    b_ = kw * (1.0 - a_);
}

// Simple first-order motor speed model implementation PWM [us] to speed [rad/s]
double prop_arm_gazebo_control::MotorSpeedModel::update(int pwm_us) noexcept
{
    double du = pwm_us - u_ref_;
    w_ = a_ * w_ + b_ * du;
    return w_;
}

// Get current speed [rad/s]
double prop_arm_gazebo_control::MotorSpeedModel::getSpeedRadSec() const noexcept
{
    return w_;
}

// Reset model state
void prop_arm_gazebo_control::MotorSpeedModel::reset() noexcept
{
    w_ = 0.0;
}

// Equality operator
bool prop_arm_gazebo_control::MotorSpeedModel::operator==(const MotorSpeedModel &other) const noexcept
{
    return (a_ == other.a_) && (b_ == other.b_) && (u_ref_ == other.u_ref_) && (w_ == other.w_);
}

#include "prop_arm_characterization/motor_speed_model.hpp"

namespace prop_arm_characterization {

namespace {

[[nodiscard]] inline std::size_t wrapIndex(long idx, std::size_t n) noexcept {
    // idx can be negative; wrap into [0, n-1]
    long m = idx % static_cast<long>(n);
    if (m < 0) {
        m += static_cast<long>(n);
    }
    return static_cast<std::size_t>(m);
}

}  // namespace

MotorSpeedModel::MotorSpeedModel(double Ts,
                                 double L_w,
                                 double c1,
                                 double c2,
                                 double d1,
                                 double d2,
                                 std::uint16_t pwm_min_us,
                                 std::uint16_t pwm_max_us,
                                 std::uint16_t pwm_spin_min_us,
                                 double motor_cmd_scale,
                                 double w0,
                                 double prop_radius_m) noexcept
    : Ts_(Ts),
      L_w_(L_w),
      c1_(c1),
      c2_(c2),
      d1_(d1),
      d2_(d2),
      pwm_min_us_(pwm_min_us),
      pwm_max_us_(pwm_max_us),
      pwm_spin_min_us_(pwm_spin_min_us),
      motor_cmd_scale_(motor_cmd_scale),
      prop_radius_m_(prop_radius_m),
      w_(w0),
      w_k_1_(w0),
      w_k_2_(w0)
{
    // Defensive defaults
    if (Ts_ <= 0.0) {
        Ts_ = 0.01;
    }
    if (L_w_ < 0.0) {
        L_w_ = 0.0;
    }

    // delay_steps = round(L_w / Ts)
    delay_steps_ = static_cast<int>(std::lround(L_w_ / Ts_));
    if (delay_steps_ < 0) {
        delay_steps_ = 0;
    }

    // Ensure PWM consistency
    if (pwm_max_us_ < pwm_min_us_) {
        std::swap(pwm_min_us_, pwm_max_us_);
    }
    if (pwm_spin_min_us_ < pwm_min_us_) {
        pwm_spin_min_us_ = pwm_min_us_;
    }
    if (pwm_spin_min_us_ > pwm_max_us_) {
        pwm_spin_min_us_ = pwm_max_us_;
    }

    // Ring buffer needs u[k - (delay_steps + 2)] access => size >= delay_steps + 3
    const std::size_t ring_size = static_cast<std::size_t>(delay_steps_ + 3);
    u_ring_.assign(ring_size, 0.0);
    write_idx_ = 0;
}

std::uint16_t MotorSpeedModel::clampPwm_(std::uint16_t pwm_us) const noexcept {
    if (pwm_us < pwm_min_us_) {
        return pwm_min_us_;
    }
    if (pwm_us > pwm_max_us_) {
        return pwm_max_us_;
    }
    return pwm_us;
}

double MotorSpeedModel::update(std::uint16_t pwm_us) noexcept {
    const std::uint16_t pwm = clampPwm_(pwm_us);

    // Deadzone + saturation relative to pwm_spin_min_us_
    double u_eff = 0.0;
    if (pwm > pwm_spin_min_us_) {
        const double raw = static_cast<double>(pwm) - static_cast<double>(pwm_spin_min_us_);
        const double u_max = static_cast<double>(pwm_max_us_) - static_cast<double>(pwm_spin_min_us_);
        u_eff = std::clamp(raw, 0.0, std::max(0.0, u_max));
    }
    u_eff *= motor_cmd_scale_;
    u_ring_[write_idx_] = u_eff;
    const std::size_t n = u_ring_.size();
    const long wi = static_cast<long>(write_idx_);
    const std::size_t idx_u1 = wrapIndex(wi - (delay_steps_ + 1), n);
    const std::size_t idx_u2 = wrapIndex(wi - (delay_steps_ + 2), n);

    const double u1 = u_ring_[idx_u1];
    const double u2 = u_ring_[idx_u2];

    // ARX(2,2) state update
    const double w_new = (c1_ * w_k_1_) + (c2_ * w_k_2_) + (d1_ * u1) + (d2_ * u2);

    // Shift output history
    w_k_2_ = w_k_1_;
    w_k_1_ = w_new;
    w_ = w_new;

    // Advance ring write index
    write_idx_ = (write_idx_ + 1U) % n;

    return w_;
}

void MotorSpeedModel::reset(double w0) noexcept {
    w_ = w0;
    w_k_1_ = w0;
    w_k_2_ = w0;

    std::fill(u_ring_.begin(), u_ring_.end(), 0.0);
    write_idx_ = 0;
}

bool MotorSpeedModel::operator==(const MotorSpeedModel& other) const noexcept {
    return Ts_ == other.Ts_ &&
           L_w_ == other.L_w_ &&
           delay_steps_ == other.delay_steps_ &&
           c1_ == other.c1_ &&
           c2_ == other.c2_ &&
           d1_ == other.d1_ &&
           d2_ == other.d2_ &&
           pwm_min_us_ == other.pwm_min_us_ &&
           pwm_max_us_ == other.pwm_max_us_ &&
           pwm_spin_min_us_ == other.pwm_spin_min_us_ &&
           motor_cmd_scale_ == other.motor_cmd_scale_ &&
           prop_radius_m_ == other.prop_radius_m_ &&
           w_ == other.w_ &&
           w_k_1_ == other.w_k_1_ &&
           w_k_2_ == other.w_k_2_;
}

}  // namespace prop_arm_characterization

#pragma once

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace prop_arm::models {

template <typename T>
struct Range {

    // Ensure the arithmetic type:
    static_assert(std::is_arithmetic_v<T>, "Range<T> requires and arithmetic type (int, float, double, ...)");

    T min{T{}};
    T max{T{}};

    // Secure function to validate the range initialization:
    void validate(const std::string& name) const {
        if(min > max) {
            std::ostringstream oss;
            oss << "Invalid range for'" << name << "': main (" << min << ") must be <= max (" << max << ")";
            throw std::runtime_error(oss.str());
        }
    }

    // Function to validate the clamp of the value:
    [[nodiscard]] bool contains(T v) const noexcept {
        return v >= min && v <= max;
    }

    // Or just clamp:
    [[nodiscard]] T clamp(T v) const noexcept {
        return std::clamp(v, min, max);
    }
};

} // namespace prop_arm::models


#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "cycle_value.hpp"
#include "libxr_def.hpp"

class YawSmc final {
 public:
  struct Param {
    float c{20.0f};
    float k{120.0f};
    float epsilon{0.5f};
    float q{21.0f};
    float p{27.0f};
    float error_deadband_rad{0.0f};
    float ftsmc_switch_rad{static_cast<float>(LibXR::PI / 180.0)};
    float sat_boundary{1.0f};
    bool ftsmc_enable{true};
  };

  static_assert(std::is_aggregate_v<Param>);
  static_assert(std::is_trivially_copyable_v<Param>);
  static_assert(std::is_standard_layout_v<Param>);

  struct Reference {
    float theta_rad{};
    float omega_rad_s{};
    float alpha_rad_s2{};
  };

  struct Feedback {
    float theta_rad{};
    float omega_rad_s{};
  };

  YawSmc(const Param& config, float torque_limit_nm)
      : config_(config), torque_limit_nm_(torque_limit_nm) {}

  [[nodiscard]] float Calculate(const Reference& reference,
                                const Feedback& feedback,
                                float j_kg_m2) const {
    const float error_rad =
        LibXR::CycleValue<float>(feedback.theta_rad) - reference.theta_rad;
    const float abs_error_rad = std::fabs(error_rad);
    if (abs_error_rad < config_.error_deadband_rad) {
      return 0.0f;
    }

    const float rate_error = feedback.omega_rad_s - reference.omega_rad_s;
    const bool use_terminal =
        config_.ftsmc_enable && config_.q > MIN_TERMINAL_EXPONENT_TERM &&
        config_.p > MIN_TERMINAL_EXPONENT_TERM && config_.q < config_.p &&
        config_.ftsmc_switch_rad > config_.error_deadband_rad &&
        abs_error_rad >= config_.ftsmc_switch_rad;
    float surface = config_.c * error_rad + rate_error;
    float equivalent_accel = -config_.c * rate_error;
    if (use_terminal) {
      const float exponent = config_.q / config_.p;
      const float signed_power =
          std::copysign(std::pow(abs_error_rad, exponent), error_rad);
      surface = rate_error + config_.c * signed_power;
      equivalent_accel =
          -config_.c * exponent * std::pow(abs_error_rad, exponent - 1.0f) *
          rate_error;
    }

    const float torque_nm =
        j_kg_m2 * (reference.alpha_rad_s2 + equivalent_accel -
                   config_.k * surface - config_.epsilon * Sat(surface));
    return std::clamp(torque_nm, -torque_limit_nm_, torque_limit_nm_);
  }

 private:
  static constexpr float MIN_TERMINAL_EXPONENT_TERM = 1e-6f;
  static constexpr float MIN_SAT_BOUNDARY = 1e-9f;

  float Sat(float surface) const {
    if (config_.sat_boundary <= MIN_SAT_BOUNDARY) {
      return static_cast<float>((surface > 0.0f) - (surface < 0.0f));
    }
    return std::clamp(surface / config_.sat_boundary, -1.0f, 1.0f);
  }

  const Param config_;
  const float torque_limit_nm_;
};

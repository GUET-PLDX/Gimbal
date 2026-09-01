#pragma once

#include <cmath>

#include "cycle_value.hpp"
#include "libxr_def.hpp"

// SI-unit port of the SMC/SMC_Tick implementation. Angles, angular rates, and
// torques are represented as rad, rad/s, and N*m respectively.
class YawSmc final {
 public:
  struct Config {
    float j_kg_m2{};
    float c{};
    float k{};
    float epsilon{};
    float q{};
    float p{};
    float error_deadband_rad{};
    float ftsmc_switch_rad{};
    float sat_boundary{};
    float torque_soft_limit_nm{};
    float torque_min_nm{};
    float torque_max_nm{};
    float torque_slew_rate_nm_s{};
    bool ftsmc_enable{};
    bool torque_slew_enable{};
  };

  struct Reference {
    // target is consumed by the code's target history; the other fields are
    // retained by the public interface but are not used by SMC_Tick math.
    float theta_rad{};
    float omega_rad_s{};
    float alpha_rad_s2{};
  };

  struct Feedback {
    float theta_rad{};
    float omega_rad_s{};
    bool valid{};
  };

  struct Output {
    // Errors and sliding surface use SI units. Torque values are N*m.
    float e_theta_rad{};
    float e_omega_rad_s{};
    float s{};
    float sat_s{};
    float tau_ff_alpha_nm{};         // J * target_ddot from SMC_Tick
    float tau_smc_nm{};              // Sliding-mode feedback term
    float tau_pre_limit_nm{};        // Core torque before all protection
    float tau_cmd_before_slew_nm{};  // After soft/hard limits, before slew
    float tau_cmd_nm{};              // Final protected command
    bool valid{};
    bool used_ftsmc{};
    bool soft_limit_active{};
    bool hard_limit_active{};
    bool slew_limit_active{};
  };

  static bool ValidateConfig(const Config& config) {
    // Keep the source implementation's permissive p/q behavior: no odd
    // integer check is added at runtime.
    if (!AllConfigFloatsFinite(config) || config.j_kg_m2 <= MIN_J_KG_M2 ||
        config.c <= 0.0f || config.k < 0.0f || config.epsilon < 0.0f ||
        config.error_deadband_rad < 0.0f || config.sat_boundary <= 0.0f) {
      return false;
    }
    if (config.ftsmc_enable &&
        (config.q <= 0.0f || config.p <= 0.0f || config.q >= config.p ||
         config.ftsmc_switch_rad <= 0.0f)) {
      return false;
    }
    if (config.torque_slew_enable && config.torque_slew_rate_nm_s <= 0.0f) {
      return false;
    }
    return true;
  }

  void Reset(float theta_rad, float omega_rad_s,
             float previous_applied_torque_nm) {
    UNUSED(omega_rad_s);
    last_applied_torque_nm_ = previous_applied_torque_nm;
    slew_anchor_torque_nm_ = previous_applied_torque_nm;
    target_last_rad_ = theta_rad;
    target_dot_rad_s_ = 0.0f;
    previous_torque_slew_enable_ = false;
  }

  Output Calculate(const Config& config, const Reference& reference,
                   const Feedback& feedback, float dt_s) {
    Output output{};
    if (!ValidateConfig(config) || !feedback.valid ||
        !std::isfinite(reference.theta_rad) ||
        !std::isfinite(reference.omega_rad_s) ||
        !std::isfinite(reference.alpha_rad_s2) ||
        !std::isfinite(feedback.theta_rad) ||
        !std::isfinite(feedback.omega_rad_s) || !std::isfinite(dt_s) ||
        dt_s <= MIN_DT_S || dt_s > MAX_DT_S) {
      return output;
    }

    // Same CycleValue shortest-path difference as the PID yaw angle loop,
    // with sliding-mode sign e = θ − θd.
    output.e_theta_rad =
        LibXR::CycleValue<float>(feedback.theta_rad) - reference.theta_rad;
    const float TARGET_DELTA_RAD =
        LibXR::CycleValue<float>(reference.theta_rad) - target_last_rad_;
    const float TARGET_DDOT_CODE = TARGET_DELTA_RAD - target_dot_rad_s_;
    output.e_omega_rad_s = feedback.omega_rad_s - target_dot_rad_s_;

    if (!std::isfinite(output.e_theta_rad) ||
        !std::isfinite(output.e_omega_rad_s)) {
      return {};
    }

    // Match SMC_Tick: target_dot is assigned before its strict deadband return,
    // while target_last advances only after a non-deadband control calculation.
    target_dot_rad_s_ = TARGET_DELTA_RAD;
    if (std::fabs(output.e_theta_rad) < config.error_deadband_rad) {
      if (config.torque_slew_enable && !previous_torque_slew_enable_) {
        slew_anchor_torque_nm_ = last_applied_torque_nm_;
      }
      previous_torque_slew_enable_ = config.torque_slew_enable;
      output.valid = true;
      return output;
    }

    output.tau_ff_alpha_nm = config.j_kg_m2 * TARGET_DDOT_CODE;

    const float ABS_E_THETA_RAD = std::fabs(output.e_theta_rad);
    // The source uses FTSMC first and replaces it with linear SMC strictly
    // below the configured SI boundary (the source literal is one unit).
    const bool USE_FTSMC =
        config.ftsmc_enable && ABS_E_THETA_RAD >= config.ftsmc_switch_rad;
    output.used_ftsmc = USE_FTSMC;

    float surface_dot_term = 0.0f;
    if (USE_FTSMC) {
      const float R = config.q / config.p;
      output.s =
          output.e_omega_rad_s + config.c * SigPow(output.e_theta_rad, R);
      // Preserve SMC_Tick's signed e_qp / abs(error) factor for negative e.
      const float E_QP = SigPow(output.e_theta_rad, R);
      surface_dot_term =
          config.c * R * output.e_omega_rad_s * E_QP / ABS_E_THETA_RAD;
    } else {
      output.s = output.e_omega_rad_s + config.c * output.e_theta_rad;
      surface_dot_term = config.c * output.e_omega_rad_s;
    }

    output.sat_s = Sat(output.s / config.sat_boundary);
    output.tau_smc_nm =
        config.j_kg_m2 * (-surface_dot_term - config.epsilon * output.sat_s -
                          config.k * output.s);
    output.tau_pre_limit_nm = output.tau_ff_alpha_nm + output.tau_smc_nm;

    if (!BaseOutputIsFinite(output)) {
      return {};
    }

    float constrained_torque_nm = output.tau_pre_limit_nm;
    if (config.torque_soft_limit_nm > 0.0f) {
      const float SOFT_LIMITED_TORQUE_NM =
          Clamp(constrained_torque_nm, -config.torque_soft_limit_nm,
                config.torque_soft_limit_nm);
      output.soft_limit_active =
          SOFT_LIMITED_TORQUE_NM != constrained_torque_nm;
      constrained_torque_nm = SOFT_LIMITED_TORQUE_NM;
    }

    const bool HARD_LIMIT_ENABLED = config.torque_min_nm < config.torque_max_nm;
    if (HARD_LIMIT_ENABLED) {
      const float HARD_LIMITED_TORQUE_NM = Clamp(
          constrained_torque_nm, config.torque_min_nm, config.torque_max_nm);
      output.hard_limit_active =
          HARD_LIMITED_TORQUE_NM != constrained_torque_nm;
      constrained_torque_nm = HARD_LIMITED_TORQUE_NM;
    }
    output.tau_cmd_before_slew_nm = constrained_torque_nm;
    output.tau_cmd_nm = output.tau_cmd_before_slew_nm;

    float next_slew_anchor_torque_nm = slew_anchor_torque_nm_;
    if (config.torque_slew_enable) {
      if (!previous_torque_slew_enable_) {
        next_slew_anchor_torque_nm = last_applied_torque_nm_;
      }

      bool limit_intersection_enabled = false;
      float limit_intersection_min_nm = 0.0f;
      float limit_intersection_max_nm = 0.0f;
      if (config.torque_soft_limit_nm > 0.0f) {
        limit_intersection_min_nm = -config.torque_soft_limit_nm;
        limit_intersection_max_nm = config.torque_soft_limit_nm;
        limit_intersection_enabled = true;
      }
      if (HARD_LIMIT_ENABLED) {
        if (!limit_intersection_enabled) {
          limit_intersection_min_nm = config.torque_min_nm;
          limit_intersection_max_nm = config.torque_max_nm;
          limit_intersection_enabled = true;
        } else {
          if (config.torque_min_nm > limit_intersection_min_nm) {
            limit_intersection_min_nm = config.torque_min_nm;
          }
          if (config.torque_max_nm < limit_intersection_max_nm) {
            limit_intersection_max_nm = config.torque_max_nm;
          }
        }
      }

      if (limit_intersection_enabled &&
          limit_intersection_min_nm <= limit_intersection_max_nm) {
        next_slew_anchor_torque_nm =
            Clamp(next_slew_anchor_torque_nm, limit_intersection_min_nm,
                  limit_intersection_max_nm);
      } else if (limit_intersection_enabled) {
        next_slew_anchor_torque_nm = output.tau_cmd_before_slew_nm;
      }

      const float MAXIMUM_TORQUE_DELTA_NM = config.torque_slew_rate_nm_s * dt_s;
      const float SLEW_MIN_NM =
          next_slew_anchor_torque_nm - MAXIMUM_TORQUE_DELTA_NM;
      const float SLEW_MAX_NM =
          next_slew_anchor_torque_nm + MAXIMUM_TORQUE_DELTA_NM;
      if (!std::isfinite(MAXIMUM_TORQUE_DELTA_NM) ||
          !std::isfinite(SLEW_MIN_NM) || !std::isfinite(SLEW_MAX_NM)) {
        return {};
      }
      output.tau_cmd_nm =
          Clamp(output.tau_cmd_before_slew_nm, SLEW_MIN_NM, SLEW_MAX_NM);
      output.slew_limit_active =
          output.tau_cmd_nm != output.tau_cmd_before_slew_nm;
    }

    if (!BaseOutputIsFinite(output)) {
      return {};
    }

    if (config.torque_slew_enable && !previous_torque_slew_enable_) {
      slew_anchor_torque_nm_ = last_applied_torque_nm_;
    }
    target_last_rad_ = reference.theta_rad;
    target_dot_rad_s_ = TARGET_DELTA_RAD;
    previous_torque_slew_enable_ = config.torque_slew_enable;
    output.valid = true;
    return output;
  }

  void CommitAppliedTorque(float applied_torque_nm) {
    if (!std::isfinite(applied_torque_nm)) {
      return;
    }
    last_applied_torque_nm_ = applied_torque_nm;
    if (previous_torque_slew_enable_) {
      slew_anchor_torque_nm_ = applied_torque_nm;
    }
  }

 private:
  static constexpr float MIN_J_KG_M2 = 1e-6f;
  static constexpr float MIN_DT_S = 0.0005f;
  static constexpr float MAX_DT_S = 0.02f;

  static float Clamp(float value, float minimum, float maximum) {
    if (value < minimum) {
      return minimum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }

  static float Sat(float y) {
    if (y > 1.0f) {
      return 1.0f;
    }
    if (y < -1.0f) {
      return -1.0f;
    }
    return y;
  }

  static float SigPow(float value, float exponent) {
    return std::copysign(std::pow(std::fabs(value), exponent), value);
  }

  static bool BaseOutputIsFinite(const Output& output) {
    return std::isfinite(output.e_theta_rad) &&
           std::isfinite(output.e_omega_rad_s) && std::isfinite(output.s) &&
           std::isfinite(output.sat_s) &&
           std::isfinite(output.tau_ff_alpha_nm) &&
           std::isfinite(output.tau_smc_nm) &&
           std::isfinite(output.tau_pre_limit_nm) &&
           std::isfinite(output.tau_cmd_before_slew_nm) &&
           std::isfinite(output.tau_cmd_nm);
  }

  static bool AllConfigFloatsFinite(const Config& config) {
    return std::isfinite(config.j_kg_m2) && std::isfinite(config.c) &&
           std::isfinite(config.k) && std::isfinite(config.epsilon) &&
           std::isfinite(config.q) && std::isfinite(config.p) &&
           std::isfinite(config.error_deadband_rad) &&
           std::isfinite(config.ftsmc_switch_rad) &&
           std::isfinite(config.sat_boundary) &&
           std::isfinite(config.torque_soft_limit_nm) &&
           std::isfinite(config.torque_min_nm) &&
           std::isfinite(config.torque_max_nm) &&
           std::isfinite(config.torque_slew_rate_nm_s);
  }

  float last_applied_torque_nm_{};
  float slew_anchor_torque_nm_{};
  float target_last_rad_{};
  float target_dot_rad_s_{};
  bool previous_torque_slew_enable_{};
};

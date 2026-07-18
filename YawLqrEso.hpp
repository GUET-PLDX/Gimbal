#pragma once

#include <cmath>
#include <cstdint>

/**
 * @brief Yaw-axis LQR/LQI controller with an extended state observer.
 *
 * The class owns controller state, while plant inertia and the hard torque
 * limit are supplied by the Gimbal module as runtime plant parameters.
 */
class YawLqrEso final {
 public:
  /** @brief Tuning and feature switches for the Yaw controller. */
  struct Config {
    float b_nms_rad{};
    float k_theta{};
    float k_omega{};
    float k_i{};
    float theta_integral_limit_rad_s{};
    float tau_coulomb_nm{};
    float coulomb_smooth_rad_s{};
    float eso_bandwidth_rad_s{};
    float eso_comp_gain{};
    float eso_comp_limit_nm{};
    float eso_omega_gate_rad_s{};
    float eso_alpha_gate_rad_s2{};
    float tau_bias_ki{};
    float tau_bias_limit_nm{};
    float tau_meas_lpf_alpha{};
    float theta_deadband_rad{};
    float torque_soft_limit_nm{};
    float torque_slew_rate_nm_s{};
    bool eso_enable{};
    bool eso_comp_enable{};
    bool coulomb_enable{};
    bool lqi_enable{};
    bool torque_bias_enable{};
    bool torque_slew_enable{};
  };

  /** @brief Desired Yaw angle, angular velocity, and angular acceleration. */
  struct Reference {
    float theta_rad{};
    float omega_rad_s{};
    float alpha_rad_s2{};
  };

  /** @brief Measured Yaw state and optional torque measurement. */
  struct Feedback {
    float theta_rad{};
    float omega_rad_s{};
    float tau_meas_nm{};
    bool valid{};
    bool torque_measurement_valid{};
  };

  /** @brief Torque command plus diagnostics from one controller update. */
  struct Output {
    float theta_unwrapped_rad{};
    float e_theta_rad{};
    float e_omega_rad_s{};
    float tau_ff_alpha_nm{};
    float tau_ff_viscous_nm{};
    float tau_ff_coulomb_nm{};
    float tau_lqi_nm{};
    float tau_lqr_nm{};
    float tau_eso_raw_nm{};
    float tau_eso_active_nm{};
    float tau_bias_nm{};
    float tau_pre_limit_nm{};
    float tau_cmd_before_slew_nm{};
    float tau_cmd_nm{};
    float z1{};
    float z2{};
    float z3{};
    bool valid{};
    bool observer_ready{};
    bool eso_comp_active{};
    bool soft_limit_active{};
    bool hard_limit_active{};
    bool slew_limit_active{};
  };

  /**
   * @brief Validate tuning and runtime plant constraints.
   * @return true when all values are finite and within supported ranges.
   */
  static bool ValidateConfig(const Config& config, float j_kg_m2,
                             float torque_limit_nm) {
    if (!AllConfigFloatsFinite(config) || !std::isfinite(j_kg_m2) ||
        !std::isfinite(torque_limit_nm) || torque_limit_nm < 0.0f ||
        j_kg_m2 <= MIN_J_KG_M2 || config.b_nms_rad < 0.0f ||
        config.k_theta < 0.0f || config.k_omega < 0.0f ||
        config.theta_deadband_rad < 0.0f) {
      return false;
    }
    if (config.eso_enable && config.eso_bandwidth_rad_s <= 0.0f) {
      return false;
    }
    if (config.eso_comp_enable &&
        (!config.eso_enable || config.eso_comp_gain < 0.0f ||
         config.eso_comp_limit_nm <= 0.0f)) {
      return false;
    }
    if (config.coulomb_enable && (config.tau_coulomb_nm < 0.0f ||
                                  config.coulomb_smooth_rad_s <= EPSILON)) {
      return false;
    }
    if (config.lqi_enable &&
        (config.k_i < 0.0f || config.theta_integral_limit_rad_s <= 0.0f)) {
      return false;
    }
    if (config.torque_bias_enable &&
        (config.tau_bias_ki < 0.0f || config.tau_bias_limit_nm <= 0.0f ||
         config.tau_meas_lpf_alpha <= 0.0f ||
         config.tau_meas_lpf_alpha > 1.0f)) {
      return false;
    }
    if (config.torque_slew_enable && config.torque_slew_rate_nm_s <= 0.0f) {
      return false;
    }
    return true;
  }

  /**
   * @brief Reset observer and actuator-history state.
   * @param previous_applied_torque_nm Last torque known to reach the motor.
   */
  void Reset(float theta_rad, float omega_rad_s,
             float previous_applied_torque_nm) {
    unwrap_raw_theta_rad_ = theta_rad;
    theta_unwrapped_rad_ = theta_rad;

    z1_ = theta_rad;
    z2_ = omega_rad_s;
    z3_ = 0.0f;
    observer_ready_ = false;
    observer_fresh_ = true;

    theta_integral_rad_s_ = 0.0f;

    tau_meas_lpf_nm_ = 0.0f;
    tau_bias_nm_ = 0.0f;

    last_applied_torque_nm_ = previous_applied_torque_nm;
    slew_anchor_torque_nm_ = previous_applied_torque_nm;

    previous_eso_enable_ = false;
    previous_eso_comp_enable_ = false;
    previous_coulomb_enable_ = false;
    previous_lqi_enable_ = false;
    previous_torque_bias_enable_ = false;
    previous_torque_slew_enable_ = false;
  }

  /**
   * @brief Calculate one bounded Yaw torque command.
   * @param j_kg_m2 Runtime Yaw plant inertia.
   * @param torque_limit_nm Symmetric hard torque limit; zero disables it.
   */
  Output Calculate(const Config& config, const Reference& reference,
                   const Feedback& feedback, float dt_s, float j_kg_m2,
                   float torque_limit_nm) {
    Output output{};
    if (!ValidateConfig(config, j_kg_m2, torque_limit_nm) || !feedback.valid ||
        !std::isfinite(reference.theta_rad) ||
        !std::isfinite(reference.omega_rad_s) ||
        !std::isfinite(reference.alpha_rad_s2) ||
        !std::isfinite(feedback.theta_rad) ||
        !std::isfinite(feedback.omega_rad_s) ||
        (config.torque_bias_enable && (!feedback.torque_measurement_valid ||
                                       !std::isfinite(feedback.tau_meas_nm))) ||
        !std::isfinite(dt_s) || dt_s <= MIN_DT_S || dt_s > MAX_DT_S) {
      return output;
    }

    const float WRAPPED_THETA_DELTA_RAD =
        WrapPi(feedback.theta_rad - unwrap_raw_theta_rad_);
    const float NEXT_THETA_UNWRAPPED_RAD =
        theta_unwrapped_rad_ + WRAPPED_THETA_DELTA_RAD;

    output.theta_unwrapped_rad = NEXT_THETA_UNWRAPPED_RAD;
    output.e_theta_rad =
        Deadband(WrapPi(feedback.theta_rad - reference.theta_rad),
                 config.theta_deadband_rad);
    output.e_omega_rad_s = feedback.omega_rad_s - reference.omega_rad_s;
    output.tau_ff_alpha_nm = j_kg_m2 * reference.alpha_rad_s2;
    output.tau_ff_viscous_nm = config.b_nms_rad * reference.omega_rad_s;

    if (!config.eso_enable) {
      z1_ = NEXT_THETA_UNWRAPPED_RAD;
      z2_ = feedback.omega_rad_s;
      z3_ = 0.0f;
      observer_ready_ = false;
      observer_fresh_ = false;
    } else if (!previous_eso_enable_ || observer_fresh_) {
      z1_ = NEXT_THETA_UNWRAPPED_RAD;
      z2_ = feedback.omega_rad_s;
      z3_ = 0.0f;
      observer_ready_ = false;
      observer_fresh_ = false;
    } else {
      const float PLANT_INPUT_GAIN = 1.0f / j_kg_m2;
      const float ESO_BANDWIDTH_SQUARED =
          config.eso_bandwidth_rad_s * config.eso_bandwidth_rad_s;
      const float ESO_BETA1 = 3.0f * config.eso_bandwidth_rad_s;
      const float ESO_BETA2 = 3.0f * ESO_BANDWIDTH_SQUARED;
      const float ESO_BETA3 =
          ESO_BANDWIDTH_SQUARED * config.eso_bandwidth_rad_s;
      const float OBSERVER_ERROR_RAD = NEXT_THETA_UNWRAPPED_RAD - z1_;
      const float ESO_Z1_DOT = z2_ + ESO_BETA1 * OBSERVER_ERROR_RAD;
      const float ESO_Z2_DOT = -(config.b_nms_rad / j_kg_m2) * z2_ +
                               PLANT_INPUT_GAIN * last_applied_torque_nm_ +
                               z3_ + ESO_BETA2 * OBSERVER_ERROR_RAD;
      const float ESO_Z3_DOT = ESO_BETA3 * OBSERVER_ERROR_RAD;
      const float ESO_Z1_CANDIDATE = z1_ + dt_s * ESO_Z1_DOT;
      const float ESO_Z2_CANDIDATE = z2_ + dt_s * ESO_Z2_DOT;
      const float ESO_Z3_CANDIDATE = z3_ + dt_s * ESO_Z3_DOT;

      if (std::isfinite(ESO_Z1_CANDIDATE) && std::isfinite(ESO_Z2_CANDIDATE) &&
          std::isfinite(ESO_Z3_CANDIDATE)) {
        z1_ = ESO_Z1_CANDIDATE;
        z2_ = ESO_Z2_CANDIDATE;
        z3_ = ESO_Z3_CANDIDATE;
        observer_ready_ = true;
      } else {
        z1_ = NEXT_THETA_UNWRAPPED_RAD;
        z2_ = feedback.omega_rad_s;
        z3_ = 0.0f;
        observer_ready_ = false;
        observer_fresh_ = true;
      }
    }

    output.tau_ff_coulomb_nm =
        config.coulomb_enable
            ? config.tau_coulomb_nm *
                  std::tanh(reference.omega_rad_s / config.coulomb_smooth_rad_s)
            : 0.0f;

    if (!config.lqi_enable) {
      theta_integral_rad_s_ = 0.0f;
    } else {
      theta_integral_rad_s_ =
          Clamp(theta_integral_rad_s_ + output.e_theta_rad * dt_s,
                -config.theta_integral_limit_rad_s,
                config.theta_integral_limit_rad_s);
    }
    output.tau_lqi_nm = -config.k_i * theta_integral_rad_s_;

    output.tau_lqr_nm = output.tau_ff_alpha_nm + output.tau_ff_viscous_nm +
                        output.tau_ff_coulomb_nm + output.tau_lqi_nm -
                        config.k_theta * output.e_theta_rad -
                        config.k_omega * output.e_omega_rad_s;

    if (config.eso_comp_enable && observer_ready_) {
      const float PLANT_INPUT_GAIN = 1.0f / j_kg_m2;
      output.tau_eso_raw_nm =
          Clamp(-config.eso_comp_gain * z3_ / PLANT_INPUT_GAIN,
                -config.eso_comp_limit_nm, config.eso_comp_limit_nm);
      const bool OMEGA_GATE_PASSED =
          config.eso_omega_gate_rad_s <= 0.0f ||
          std::fabs(feedback.omega_rad_s) <= config.eso_omega_gate_rad_s;
      const bool ALPHA_GATE_PASSED =
          config.eso_alpha_gate_rad_s2 <= 0.0f ||
          std::fabs(reference.alpha_rad_s2) <= config.eso_alpha_gate_rad_s2;
      if (OMEGA_GATE_PASSED && ALPHA_GATE_PASSED) {
        output.tau_eso_active_nm = output.tau_eso_raw_nm;
        output.eso_comp_active = true;
      }
    }

    const float TORQUE_WITHOUT_BIAS_NM =
        output.tau_lqr_nm + output.tau_eso_active_nm;
    if (!config.torque_bias_enable) {
      tau_meas_lpf_nm_ = 0.0f;
      tau_bias_nm_ = 0.0f;
    } else {
      if (!previous_torque_bias_enable_) {
        tau_meas_lpf_nm_ = feedback.tau_meas_nm;
      } else {
        tau_meas_lpf_nm_ += config.tau_meas_lpf_alpha *
                            (feedback.tau_meas_nm - tau_meas_lpf_nm_);
      }
      tau_bias_nm_ = Clamp(
          tau_bias_nm_ + config.tau_bias_ki *
                             (TORQUE_WITHOUT_BIAS_NM - tau_meas_lpf_nm_) * dt_s,
          -config.tau_bias_limit_nm, config.tau_bias_limit_nm);
    }
    output.tau_bias_nm = tau_bias_nm_;
    output.tau_pre_limit_nm = TORQUE_WITHOUT_BIAS_NM + output.tau_bias_nm;
    output.z1 = z1_;
    output.z2 = z2_;
    output.z3 = z3_;
    output.observer_ready = observer_ready_;

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

    const bool HARD_LIMIT_ENABLED = torque_limit_nm > 0.0f;
    if (HARD_LIMIT_ENABLED) {
      const float HARD_LIMITED_TORQUE_NM =
          Clamp(constrained_torque_nm, -torque_limit_nm, torque_limit_nm);
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
          limit_intersection_min_nm = -torque_limit_nm;
          limit_intersection_max_nm = torque_limit_nm;
          limit_intersection_enabled = true;
        } else {
          if (-torque_limit_nm > limit_intersection_min_nm) {
            limit_intersection_min_nm = -torque_limit_nm;
          }
          if (torque_limit_nm < limit_intersection_max_nm) {
            limit_intersection_max_nm = torque_limit_nm;
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

      const float MAX_TORQUE_DELTA_NM = config.torque_slew_rate_nm_s * dt_s;
      const float SLEW_MIN_NM =
          next_slew_anchor_torque_nm - MAX_TORQUE_DELTA_NM;
      const float SLEW_MAX_NM =
          next_slew_anchor_torque_nm + MAX_TORQUE_DELTA_NM;
      if (!std::isfinite(MAX_TORQUE_DELTA_NM) || !std::isfinite(SLEW_MIN_NM) ||
          !std::isfinite(SLEW_MAX_NM)) {
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

    unwrap_raw_theta_rad_ = feedback.theta_rad;
    theta_unwrapped_rad_ = NEXT_THETA_UNWRAPPED_RAD;
    if (config.torque_slew_enable && !previous_torque_slew_enable_) {
      slew_anchor_torque_nm_ = last_applied_torque_nm_;
    }
    previous_eso_enable_ = config.eso_enable;
    previous_eso_comp_enable_ = config.eso_comp_enable;
    previous_coulomb_enable_ = config.coulomb_enable;
    previous_lqi_enable_ = config.lqi_enable;
    previous_torque_bias_enable_ = config.torque_bias_enable;
    previous_torque_slew_enable_ = config.torque_slew_enable;
    output.valid = true;
    return output;
  }

  /** @brief Commit the torque that the motor actually accepted. */
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
  static constexpr float EPSILON = 1e-6f;
  static constexpr float PI = 3.14159265358979323846f;
  static constexpr float TWO_PI = 2.0f * PI;

  /** @brief Clamp a scalar to an inclusive range. */
  static float Clamp(float value, float minimum, float maximum) {
    if (value < minimum) {
      return minimum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }

  /** @brief Wrap an angle to [-pi, pi). */
  static float WrapPi(float angle_rad) {
    float wrapped_rad = std::fmod(angle_rad + PI, TWO_PI);
    if (wrapped_rad < 0.0f) {
      wrapped_rad += TWO_PI;
    }
    return wrapped_rad - PI;
  }

  /** @brief Remove a symmetric deadband from a scalar error. */
  static float Deadband(float value, float deadband) {
    if (value > deadband) {
      return value - deadband;
    }
    if (value < -deadband) {
      return value + deadband;
    }
    return 0.0f;
  }

  /** @brief Check the numerical diagnostics before publishing an output. */
  static bool BaseOutputIsFinite(const Output& output) {
    return std::isfinite(output.theta_unwrapped_rad) &&
           std::isfinite(output.e_theta_rad) &&
           std::isfinite(output.e_omega_rad_s) &&
           std::isfinite(output.tau_ff_alpha_nm) &&
           std::isfinite(output.tau_ff_viscous_nm) &&
           std::isfinite(output.tau_ff_coulomb_nm) &&
           std::isfinite(output.tau_lqi_nm) &&
           std::isfinite(output.tau_lqr_nm) &&
           std::isfinite(output.tau_eso_raw_nm) &&
           std::isfinite(output.tau_eso_active_nm) &&
           std::isfinite(output.tau_bias_nm) &&
           std::isfinite(output.tau_pre_limit_nm) &&
           std::isfinite(output.tau_cmd_before_slew_nm) &&
           std::isfinite(output.tau_cmd_nm);
  }

  /** @brief Check all floating-point tuning values for NaN or infinity. */
  static bool AllConfigFloatsFinite(const Config& config) {
    return std::isfinite(config.b_nms_rad) && std::isfinite(config.k_theta) &&
           std::isfinite(config.k_omega) && std::isfinite(config.k_i) &&
           std::isfinite(config.theta_integral_limit_rad_s) &&
           std::isfinite(config.tau_coulomb_nm) &&
           std::isfinite(config.coulomb_smooth_rad_s) &&
           std::isfinite(config.eso_bandwidth_rad_s) &&
           std::isfinite(config.eso_comp_gain) &&
           std::isfinite(config.eso_comp_limit_nm) &&
           std::isfinite(config.eso_omega_gate_rad_s) &&
           std::isfinite(config.eso_alpha_gate_rad_s2) &&
           std::isfinite(config.tau_bias_ki) &&
           std::isfinite(config.tau_bias_limit_nm) &&
           std::isfinite(config.tau_meas_lpf_alpha) &&
           std::isfinite(config.theta_deadband_rad) &&
           std::isfinite(config.torque_soft_limit_nm) &&
           std::isfinite(config.torque_slew_rate_nm_s);
  }

  float unwrap_raw_theta_rad_{};
  float theta_unwrapped_rad_{};

  float z1_{};
  float z2_{};
  float z3_{};
  bool observer_ready_{};
  bool observer_fresh_{};

  float theta_integral_rad_s_{};

  float tau_meas_lpf_nm_{};
  float tau_bias_nm_{};

  float last_applied_torque_nm_{};
  float slew_anchor_torque_nm_{};

  bool previous_eso_enable_{};
  bool previous_eso_comp_enable_{};
  bool previous_coulomb_enable_{};
  bool previous_lqi_enable_{};
  bool previous_torque_bias_enable_{};
  bool previous_torque_slew_enable_{};
};

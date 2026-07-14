#pragma once

#include <cmath>

class YawLqrEso final {
 public:
  struct Config {
    float j_kg_m2{};
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
    float torque_min_nm{};
    float torque_max_nm{};
    float torque_slew_rate_nm_s{};
    bool eso_enable{};
    bool eso_comp_enable{};
    bool coulomb_enable{};
    bool lqi_enable{};
    bool torque_bias_enable{};
    bool torque_slew_enable{};
  };

  struct Reference {
    float theta_rad{};
    float omega_rad_s{};
    float alpha_rad_s2{};
  };

  struct Feedback {
    float theta_rad{};
    float omega_rad_s{};
    float tau_meas_nm{};
    bool valid{};
    bool torque_measurement_valid{};
  };

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

  static bool ValidateConfig(const Config& config) {
    if (!AllConfigFloatsFinite(config) || config.j_kg_m2 <= MIN_J_KG_M2 ||
        config.b_nms_rad < 0.0f || config.k_theta < 0.0f ||
        config.k_omega < 0.0f || config.theta_deadband_rad < 0.0f) {
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

  void Reset(float theta_rad, float omega_rad_s,
             float previous_applied_torque_nm) {
    unwrap_raw_theta_rad_ = theta_rad;
    theta_unwrapped_rad_ = theta_rad;

    z1_ = theta_rad;
    z2_ = omega_rad_s;
    z3_ = 0.0f;
    observer_ready_ = false;
    observer_fresh_ = false;

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

    const float THETA_DELTA_RAD =
        WrapPi(feedback.theta_rad - unwrap_raw_theta_rad_);
    const float NEXT_THETA_UNWRAPPED_RAD =
        theta_unwrapped_rad_ + THETA_DELTA_RAD;

    output.theta_unwrapped_rad = NEXT_THETA_UNWRAPPED_RAD;
    output.e_theta_rad =
        Deadband(WrapPi(feedback.theta_rad - reference.theta_rad),
                 config.theta_deadband_rad);
    output.e_omega_rad_s = feedback.omega_rad_s - reference.omega_rad_s;
    output.tau_ff_alpha_nm = config.j_kg_m2 * reference.alpha_rad_s2;
    output.tau_ff_viscous_nm = config.b_nms_rad * reference.omega_rad_s;
    output.tau_lqr_nm = output.tau_ff_alpha_nm + output.tau_ff_viscous_nm -
                        config.k_theta * output.e_theta_rad -
                        config.k_omega * output.e_omega_rad_s;
    output.tau_pre_limit_nm = output.tau_lqr_nm;

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

    unwrap_raw_theta_rad_ = feedback.theta_rad;
    theta_unwrapped_rad_ = NEXT_THETA_UNWRAPPED_RAD;
    if (config.torque_slew_enable && !previous_torque_slew_enable_) {
      slew_anchor_torque_nm_ = last_applied_torque_nm_;
    }
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
  static constexpr float EPSILON = 1e-6f;
  static constexpr float PI = 3.14159265358979323846f;
  static constexpr float TWO_PI = 2.0f * PI;

  static float Clamp(float value, float minimum, float maximum) {
    if (value < minimum) {
      return minimum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }

  static float WrapPi(float angle_rad) {
    float wrapped_rad = std::fmod(angle_rad + PI, TWO_PI);
    if (wrapped_rad < 0.0f) {
      wrapped_rad += TWO_PI;
    }
    return wrapped_rad - PI;
  }

  static float Deadband(float value, float deadband) {
    if (value > deadband) {
      return value - deadband;
    }
    if (value < -deadband) {
      return value + deadband;
    }
    return 0.0f;
  }

  static bool BaseOutputIsFinite(const Output& output) {
    return std::isfinite(output.theta_unwrapped_rad) &&
           std::isfinite(output.e_theta_rad) &&
           std::isfinite(output.e_omega_rad_s) &&
           std::isfinite(output.tau_ff_alpha_nm) &&
           std::isfinite(output.tau_ff_viscous_nm) &&
           std::isfinite(output.tau_lqr_nm) &&
           std::isfinite(output.tau_pre_limit_nm) &&
           std::isfinite(output.tau_cmd_before_slew_nm) &&
           std::isfinite(output.tau_cmd_nm);
  }

  static bool AllConfigFloatsFinite(const Config& config) {
    return std::isfinite(config.j_kg_m2) && std::isfinite(config.b_nms_rad) &&
           std::isfinite(config.k_theta) && std::isfinite(config.k_omega) &&
           std::isfinite(config.k_i) &&
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
           std::isfinite(config.torque_min_nm) &&
           std::isfinite(config.torque_max_nm) &&
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

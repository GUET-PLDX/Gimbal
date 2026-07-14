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
             float previous_applied_torque_nm);
  Output Calculate(const Config& config, const Reference& reference,
                   const Feedback& feedback, float dt_s);
  void CommitAppliedTorque(float applied_torque_nm);

 private:
  static constexpr float MIN_J_KG_M2 = 1e-6f;
  static constexpr float MIN_DT_S = 0.0005f;
  static constexpr float MAX_DT_S = 0.02f;
  static constexpr float EPSILON = 1e-6f;

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

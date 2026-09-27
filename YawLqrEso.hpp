#pragma once

#include <cmath>
#include <concepts>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief Yaw-axis LQR/LQI controller with an extended state observer.
 *
 * The class owns controller state, while plant inertia and the hard torque
 * limit are supplied by the Gimbal module as runtime plant parameters.
 */
class YawLqrEso final {
 public:
  /**
   * @brief Tuning and feature switches for the Yaw controller.
   *
   * @note 默认值**必须与 `Gimbal.hpp` 里 `yaw_lqr_eso` 的 manifest
   * 默认值逐字段一致** （`tests/gimbal_config_order_regression.py`
   * 会比对，分叉即失败）。 manifest
   * 的值是**建议基线**，不是实车整定值；实车值在 `User/RobotConfig/`
   * 下的机器人配置里按位置覆盖。
   * @note 默认值**不是"未配置"哨兵**：模块不再给 `yaw_lqr_eso` 形参默认实参，
   *       漏配会在编译期报错，因此这里放的是可用配置而不是全零。
   *       但显式构造的零增益配置仍必须被 `ValidateConfig` 拒绝（`k_theta` /
   *       `k_omega` 为 0 时角度环与角速度环同时失效 → 静默零力矩）。
   */
  struct Config {
    float k_theta{1.0f};
    float k_omega{1.0f};
    float k_i{0.2f};
    float theta_integral_limit_rad_s{0.5f};
    float tau_coulomb_nm{0.05f};
    float coulomb_smooth_rad_s{0.2f};
    float eso_bandwidth_rad_s{30.0f};
    float eso_comp_gain{1.0f};
    float eso_comp_limit_nm{0.3f};
    float eso_omega_gate_rad_s{5.0f};
    float eso_alpha_gate_rad_s2{50.0f};
    float tau_bias_ki{0.5f};
    float tau_bias_limit_nm{0.15f};
    float tau_meas_lpf_alpha{0.1f};
    float theta_deadband_rad{0.0f};
    float torque_soft_limit_nm{2.0f};
    float torque_slew_rate_nm_s{1000.0f};
    bool eso_enable{true};
    bool eso_comp_enable{false};
    bool coulomb_enable{false};
    bool lqi_enable{false};
    bool torque_bias_enable{false};
    bool torque_slew_enable{true};
  };

  // xrobot 按声明顺序位置聚合 Config，这三条契约守护该初始化方式。
  static_assert(std::is_aggregate_v<Config>);
  static_assert(std::is_trivially_copyable_v<Config>);
  static_assert(std::is_standard_layout_v<Config>);

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
   * @brief 用锁存的配置构造控制器。
   * @note 不提供默认构造：控制器不能在没有显式配置的情况下存在。
   *       完美转发对齐 LibXR::PID 的同形构造；requires 约束同时阻止该模板
   *       劫持隐式拷贝构造（`YawLqrEso a(b);` 的 ConfigType 是 `const
   * YawLqrEso&`）。
   */
  template <typename ConfigType>
    requires std::same_as<std::remove_cvref_t<ConfigType>, Config>
  explicit YawLqrEso(ConfigType&& config)
      : config_(std::forward<ConfigType>(config)) {
    Reset(0.0f, 0.0f, 0.0f);
  }

  /// @brief 只读配置访问器（诊断与主机测试用）。
  [[nodiscard]] const Config& GetConfig() const noexcept { return config_; }

  /**
   * @brief Validate tuning and runtime plant constraints.
   * @return true when all values are finite and within supported ranges.
   * @note `k_theta` / `k_omega` 必须**严格为正**：两者同时为
   * 0（全零配置的必然结果）
   *       会让角度环与角速度环同时失效，控制器静默输出零力矩。
   *       该门槛是"配置结构体被整块清零 /
   * 聚合实参被截断"这类误配的最后一道防线。
   */
  static bool ValidateConfig(const Config& config, float j_kg_m2,
                             float torque_limit_nm, float b_nms_rad) {
    if (!AllConfigFloatsFinite(config) || !std::isfinite(j_kg_m2) ||
        !std::isfinite(torque_limit_nm) || !std::isfinite(b_nms_rad) ||
        torque_limit_nm < 0.0f || j_kg_m2 <= MIN_J_KG_M2 || b_nms_rad < 0.0f ||
        config.k_theta <= 0.0f || config.k_omega <= 0.0f ||
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

    bias_primed_ = false;
    slew_primed_ = false;
  }

  /**
   * @brief Calculate one bounded Yaw torque command.
   * @param j_kg_m2 Runtime Yaw plant inertia.
   * @param torque_limit_nm Symmetric hard torque limit; zero disables it.
   */
  [[nodiscard]] Output Calculate(const Reference& reference,
                                 const Feedback& feedback, float dt_s,
                                 float j_kg_m2, float torque_limit_nm,
                                 float b_nms_rad) {
    Output output{};
    if (!ValidateConfig(config_, j_kg_m2, torque_limit_nm, b_nms_rad) ||
        !feedback.valid || !std::isfinite(reference.theta_rad) ||
        !std::isfinite(reference.omega_rad_s) ||
        !std::isfinite(reference.alpha_rad_s2) ||
        !std::isfinite(feedback.theta_rad) ||
        !std::isfinite(feedback.omega_rad_s) ||
        (config_.torque_bias_enable &&
         (!feedback.torque_measurement_valid ||
          !std::isfinite(feedback.tau_meas_nm))) ||
        !std::isfinite(dt_s) || dt_s <= MIN_DT_S || dt_s > MAX_DT_S) {
      return output;
    }

    const float WRAPPED_THETA_DELTA_RAD =
        LibXR::CycleValue<float>(feedback.theta_rad) - unwrap_raw_theta_rad_;
    const float NEXT_THETA_UNWRAPPED_RAD =
        theta_unwrapped_rad_ + WRAPPED_THETA_DELTA_RAD;

    output.theta_unwrapped_rad = NEXT_THETA_UNWRAPPED_RAD;
    output.e_theta_rad = Deadband(
        LibXR::CycleValue<float>(feedback.theta_rad) - reference.theta_rad,
        config_.theta_deadband_rad);
    output.e_omega_rad_s = feedback.omega_rad_s - reference.omega_rad_s;
    output.tau_ff_alpha_nm = j_kg_m2 * reference.alpha_rad_s2;
    output.tau_ff_viscous_nm = b_nms_rad * reference.omega_rad_s;

    if (!config_.eso_enable) {
      z1_ = NEXT_THETA_UNWRAPPED_RAD;
      z2_ = feedback.omega_rad_s;
      z3_ = 0.0f;
      observer_ready_ = false;
      observer_fresh_ = false;
    } else if (observer_fresh_) {
      // 配置锁存后 ESO 不会在运行期被重新使能，"是否首周期"完全由
      // observer_fresh_（Reset 或上一周期数值异常时置位）表达。
      z1_ = NEXT_THETA_UNWRAPPED_RAD;
      z2_ = feedback.omega_rad_s;
      z3_ = 0.0f;
      observer_ready_ = false;
      observer_fresh_ = false;
    } else {
      const float PLANT_INPUT_GAIN = 1.0f / j_kg_m2;
      const float ESO_BANDWIDTH_SQUARED =
          config_.eso_bandwidth_rad_s * config_.eso_bandwidth_rad_s;
      const float ESO_BETA1 = 3.0f * config_.eso_bandwidth_rad_s;
      const float ESO_BETA2 = 3.0f * ESO_BANDWIDTH_SQUARED;
      const float ESO_BETA3 =
          ESO_BANDWIDTH_SQUARED * config_.eso_bandwidth_rad_s;
      const float OBSERVER_ERROR_RAD = NEXT_THETA_UNWRAPPED_RAD - z1_;
      const float ESO_Z1_DOT = z2_ + ESO_BETA1 * OBSERVER_ERROR_RAD;
      const float ESO_Z2_DOT = -(b_nms_rad / j_kg_m2) * z2_ +
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
        config_.coulomb_enable
            ? config_.tau_coulomb_nm * std::tanh(reference.omega_rad_s /
                                                 config_.coulomb_smooth_rad_s)
            : 0.0f;

    if (!config_.lqi_enable) {
      theta_integral_rad_s_ = 0.0f;
    } else {
      theta_integral_rad_s_ =
          Clamp(theta_integral_rad_s_ + output.e_theta_rad * dt_s,
                -config_.theta_integral_limit_rad_s,
                config_.theta_integral_limit_rad_s);
    }
    output.tau_lqi_nm = -config_.k_i * theta_integral_rad_s_;

    output.tau_lqr_nm = output.tau_ff_alpha_nm + output.tau_ff_viscous_nm +
                        output.tau_ff_coulomb_nm + output.tau_lqi_nm -
                        config_.k_theta * output.e_theta_rad -
                        config_.k_omega * output.e_omega_rad_s;

    if (config_.eso_comp_enable && observer_ready_) {
      const float PLANT_INPUT_GAIN = 1.0f / j_kg_m2;
      output.tau_eso_raw_nm =
          Clamp(-config_.eso_comp_gain * z3_ / PLANT_INPUT_GAIN,
                -config_.eso_comp_limit_nm, config_.eso_comp_limit_nm);
      const bool OMEGA_GATE_PASSED =
          config_.eso_omega_gate_rad_s <= 0.0f ||
          std::fabs(feedback.omega_rad_s) <= config_.eso_omega_gate_rad_s;
      const bool ALPHA_GATE_PASSED =
          config_.eso_alpha_gate_rad_s2 <= 0.0f ||
          std::fabs(reference.alpha_rad_s2) <= config_.eso_alpha_gate_rad_s2;
      if (OMEGA_GATE_PASSED && ALPHA_GATE_PASSED) {
        output.tau_eso_active_nm = output.tau_eso_raw_nm;
        output.eso_comp_active = true;
      }
    }

    const float TORQUE_WITHOUT_BIAS_NM =
        output.tau_lqr_nm + output.tau_eso_active_nm;
    if (!config_.torque_bias_enable) {
      tau_meas_lpf_nm_ = 0.0f;
      tau_bias_nm_ = 0.0f;
    } else {
      if (!bias_primed_) {
        tau_meas_lpf_nm_ = feedback.tau_meas_nm;
      } else {
        tau_meas_lpf_nm_ += config_.tau_meas_lpf_alpha *
                            (feedback.tau_meas_nm - tau_meas_lpf_nm_);
      }
      tau_bias_nm_ = Clamp(
          tau_bias_nm_ + config_.tau_bias_ki *
                             (TORQUE_WITHOUT_BIAS_NM - tau_meas_lpf_nm_) * dt_s,
          -config_.tau_bias_limit_nm, config_.tau_bias_limit_nm);
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
    if (config_.torque_soft_limit_nm > 0.0f) {
      const float SOFT_LIMITED_TORQUE_NM =
          Clamp(constrained_torque_nm, -config_.torque_soft_limit_nm,
                config_.torque_soft_limit_nm);
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
    if (config_.torque_slew_enable) {
      if (!slew_primed_) {
        next_slew_anchor_torque_nm = last_applied_torque_nm_;
      }

      bool limit_intersection_enabled = false;
      float limit_intersection_min_nm = 0.0f;
      float limit_intersection_max_nm = 0.0f;
      if (config_.torque_soft_limit_nm > 0.0f) {
        limit_intersection_min_nm = -config_.torque_soft_limit_nm;
        limit_intersection_max_nm = config_.torque_soft_limit_nm;
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

      const float MAX_TORQUE_DELTA_NM = config_.torque_slew_rate_nm_s * dt_s;
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
    if (config_.torque_slew_enable && !slew_primed_) {
      slew_anchor_torque_nm_ = last_applied_torque_nm_;
    }
    bias_primed_ = true;
    slew_primed_ = true;
    output.valid = true;
    return output;
  }

  /** @brief Commit the torque that the motor actually accepted. */
  void CommitAppliedTorque(float applied_torque_nm) {
    if (!std::isfinite(applied_torque_nm)) {
      return;
    }
    last_applied_torque_nm_ = applied_torque_nm;
    // 配置锁存后 slew 是否启用是常量，故只需判「Calculate 是否已跑过一周期」。
    if (slew_primed_ && config_.torque_slew_enable) {
      slew_anchor_torque_nm_ = applied_torque_nm;
    }
  }

 private:
  const Config config_;  ///< 构造期锁存，运行期只读（对齐 LibXR::PID::param_）

  static constexpr float MIN_J_KG_M2 = 1e-6f;
  static constexpr float MIN_DT_S = 0.0005f;
  static constexpr float MAX_DT_S = 0.02f;
  static constexpr float EPSILON = 1e-6f;

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
    return std::isfinite(config.k_theta) && std::isfinite(config.k_omega) &&
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

  bool bias_primed_{};  ///< 首周期播种 tau_meas_lpf_nm_ 的标志
  bool slew_primed_{};  ///< 首周期播种 slew anchor 的标志
};

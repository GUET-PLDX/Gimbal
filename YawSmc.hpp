#pragma once

#include <cmath>
#include <concepts>
#include <type_traits>
#include <utility>

#include "cycle_value.hpp"
#include "libxr_def.hpp"

// SI-unit port of the SMC/SMC_Tick implementation. Angles, angular rates, and
// torques are represented as rad, rad/s, and N*m respectively.
class YawSmc final {
 public:
  /**
   * @brief Yaw 滑模调参与功能开关。
   *
   * @note 默认值**必须与 `Gimbal.hpp` 里 `yaw_smc` 的 manifest
   * 默认值逐字段一致** （`tests/gimbal_config_order_regression.py`
   * 会比对，分叉即失败）。 manifest
   * 的值是**建议基线**，不是实车整定值；实车值在 `User/RobotConfig/`
   * 下的机器人配置里按位置覆盖。
   * @note 默认值**不是"未配置"哨兵**：模块不再给 `yaw_smc` 形参默认实参，
   *       漏配会在编译期报错，因此这里放的是可用配置而不是全零。
   */
  struct Config {
    float c{20.0f};
    float k{120.0f};
    float epsilon{0.5f};
    float q{21.0f};
    float p{27.0f};
    float error_deadband_rad{0.0f};
    float ftsmc_switch_rad{static_cast<float>(LibXR::PI / 180.0)};
    float sat_boundary{1.0f};
    float torque_soft_limit_nm{2.0f};
    float torque_min_nm{-2.223f};
    float torque_max_nm{2.223f};
    float torque_slew_rate_nm_s{1000.0f};
    bool ftsmc_enable{true};
    bool torque_slew_enable{true};
  };

  // xrobot 按声明顺序位置聚合 Config，这三条契约守护该初始化方式。
  static_assert(std::is_aggregate_v<Config>);
  static_assert(std::is_trivially_copyable_v<Config>);
  static_assert(std::is_standard_layout_v<Config>);

  struct Reference {
    // 目标角度、角速度和角加速度均使用 SI 单位并直接参与控制律。
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

  /**
   * @brief 用锁存的配置构造控制器。
   * @note 不提供默认构造：控制器不能在没有显式配置的情况下存在。
   *       完美转发对齐 LibXR::PID 的同形构造；requires 约束同时阻止该模板
   *       劫持隐式拷贝构造（`YawSmc a(b);` 的 ConfigType 是 `const YawSmc&`）。
   */
  template <typename ConfigType>
    requires std::same_as<std::remove_cvref_t<ConfigType>, Config>
  explicit YawSmc(ConfigType&& config)
      : config_(std::forward<ConfigType>(config)) {
    Reset(0.0f, 0.0f, 0.0f);
  }

  /// @brief 只读配置访问器（诊断与主机测试用）。
  [[nodiscard]] const Config& GetConfig() const noexcept { return config_; }

  static bool ValidateConfig(const Config& config, float j_kg_m2) {
    // Keep the source implementation's permissive p/q behavior: no odd
    // integer check is added at runtime.
    if (!AllConfigFloatsFinite(config) || !std::isfinite(j_kg_m2) ||
        j_kg_m2 <= MIN_J_KG_M2 || config.c <= 0.0f || config.k < 0.0f ||
        config.epsilon < 0.0f || config.error_deadband_rad < 0.0f ||
        config.sat_boundary <= 0.0f) {
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
    UNUSED(theta_rad);
    UNUSED(omega_rad_s);
    last_applied_torque_nm_ = previous_applied_torque_nm;
    slew_anchor_torque_nm_ = previous_applied_torque_nm;
    slew_primed_ = false;
  }

  [[nodiscard]] Output Calculate(const Reference& reference,
                                 const Feedback& feedback, float dt_s,
                                 float j_kg_m2) {
    Output output{};
    if (!ValidateConfig(config_, j_kg_m2) || !feedback.valid ||
        !std::isfinite(reference.theta_rad) ||
        !std::isfinite(reference.omega_rad_s) ||
        !std::isfinite(reference.alpha_rad_s2) ||
        !std::isfinite(feedback.theta_rad) ||
        !std::isfinite(feedback.omega_rad_s) || !std::isfinite(dt_s) ||
        dt_s <= MIN_DT_S || dt_s > MAX_DT_S) {
      return output;
    }

    // 与 Yaw PID 角度环一致，使用 CycleValue 最短路并定义 e = θ - θd。
    output.e_theta_rad =
        LibXR::CycleValue<float>(feedback.theta_rad) - reference.theta_rad;
    output.e_omega_rad_s = feedback.omega_rad_s - reference.omega_rad_s;

    if (!std::isfinite(output.e_theta_rad) ||
        !std::isfinite(output.e_omega_rad_s)) {
      return {};
    }

    if (std::fabs(output.e_theta_rad) < config_.error_deadband_rad) {
      if (config_.torque_slew_enable && !slew_primed_) {
        slew_anchor_torque_nm_ = last_applied_torque_nm_;
      }
      slew_primed_ = true;
      output.valid = true;
      return output;
    }

    output.tau_ff_alpha_nm = j_kg_m2 * reference.alpha_rad_s2;

    const float ABS_E_THETA_RAD = std::fabs(output.e_theta_rad);
    // 误差达到配置边界时使用 FTSMC，小于边界时使用线性 SMC。
    const bool USE_FTSMC =
        config_.ftsmc_enable && ABS_E_THETA_RAD >= config_.ftsmc_switch_rad;
    output.used_ftsmc = USE_FTSMC;

    float surface_dot_term = 0.0f;
    if (USE_FTSMC) {
      const float R = config_.q / config_.p;
      output.s =
          output.e_omega_rad_s + config_.c * SigPow(output.e_theta_rad, R);
      // d(sig^r(e))/dt = r * |e|^(r-1) * e_dot，不含 sign(e)。
      surface_dot_term = config_.c * R * std::pow(ABS_E_THETA_RAD, R - 1.0f) *
                         output.e_omega_rad_s;
    } else {
      output.s = output.e_omega_rad_s + config_.c * output.e_theta_rad;
      surface_dot_term = config_.c * output.e_omega_rad_s;
    }

    output.sat_s = Sat(output.s / config_.sat_boundary);
    output.tau_smc_nm =
        j_kg_m2 * (-surface_dot_term - config_.epsilon * output.sat_s -
                   config_.k * output.s);
    output.tau_pre_limit_nm = output.tau_ff_alpha_nm + output.tau_smc_nm;

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

    const bool HARD_LIMIT_ENABLED =
        config_.torque_min_nm < config_.torque_max_nm;
    if (HARD_LIMIT_ENABLED) {
      const float HARD_LIMITED_TORQUE_NM = Clamp(
          constrained_torque_nm, config_.torque_min_nm, config_.torque_max_nm);
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
          limit_intersection_min_nm = config_.torque_min_nm;
          limit_intersection_max_nm = config_.torque_max_nm;
          limit_intersection_enabled = true;
        } else {
          if (config_.torque_min_nm > limit_intersection_min_nm) {
            limit_intersection_min_nm = config_.torque_min_nm;
          }
          if (config_.torque_max_nm < limit_intersection_max_nm) {
            limit_intersection_max_nm = config_.torque_max_nm;
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

      const float MAXIMUM_TORQUE_DELTA_NM =
          config_.torque_slew_rate_nm_s * dt_s;
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

    if (config_.torque_slew_enable && !slew_primed_) {
      slew_anchor_torque_nm_ = last_applied_torque_nm_;
    }
    slew_primed_ = true;
    output.valid = true;
    return output;
  }

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
    return std::isfinite(config.c) && std::isfinite(config.k) &&
           std::isfinite(config.epsilon) && std::isfinite(config.q) &&
           std::isfinite(config.p) &&
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
  bool slew_primed_{};  ///< 首周期 slew anchor 播种标志
};

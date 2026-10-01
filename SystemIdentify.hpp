#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "libxr_def.hpp"

// 云台 Pitch/Yaw 轴 J/B 在线辨识（一次性标定工具）。
//
// 被控对象模型（《单轴云台最优控制器设计》第 1 节）：
//     J * alpha + B * omega + tau_c * tanh(omega / w0) [+ mgl * sin(theta)] =
//     tau
// 激励为开环力矩双向阶跃序列（方波），RLS 带遗忘因子递推估计参数。
// 本类与 LibXR 完全解耦（仅依赖标准库），可在 PC 端直接实例化做仿真测试。

/// @brief 辨识轴选择（yaml/manifest 以 IdentifyAxis::XXX 形式按位置传入）。
enum class IdentifyAxis : uint8_t { YAW_ONLY, PITCH_ONLY, BOTH };

/// @brief 辨识状态机。
enum class IdentifyState : uint8_t {
  IDLE,
  ARMING,
  YAW_EXCITE,
  YAW_SETTLE,
  PITCH_EXCITE,
  PITCH_SETTLE,
  DONE,
  ABORTED
};

/// @brief 中止原因（Ozone 可读）。
enum class IdentifyAbort : uint8_t {
  NONE,
  MOTOR_OFFLINE,
  OMEGA_LIMIT,
  PITCH_ANGLE_LIMIT,
  AXIS_TIMEOUT,
  EXTERNAL_EVENT,
  NUMERICAL_DIVERGENCE
};

/// @brief 标量展开的递推最小二乘（带遗忘因子），数学上与
/// RM2024-PowerModule 的 RLS<dim> 等价，但无 Matrix/xTask 依赖。
/// 每步对 P 做对称化以抑制 float 非对称漂移导致的正定性破坏。
template <uint32_t dim>
class RlsIdent final {
 public:
  RlsIdent() = delete;

  RlsIdent(float delta, float lambda) { Reset(delta, lambda); }

  void Reset(float delta, float lambda) {
    lambda_ = lambda;
    for (uint32_t i = 0; i < dim; i++) {
      theta_[i] = 0.0f;
      for (uint32_t j = 0; j < dim; j++) {
        p_[i][j] = (i == j) ? delta : 0.0f;
      }
    }
  }

  /// @brief 递推一步：K = P*phi / (lambda + phi'P*phi)；theta +=
  /// K*(y-phi'*theta)。 P 采用 Joseph 形式 [(I-K*phi')P(I-K*phi')' +
  /// K*K']/lambda，构造上保证 半正定——朴素形式 (P-K*phi'*P)/lambda 在 P
  /// 收缩后发生灾难性抵消， float 下失去正定性导致增益反号、参数发散（仿真实测
  /// J 发散至 1e6 量级）。 冻结期（门控不满足）由调用方决定不调用本函数，P 与
  /// theta 完全不动。
  void Update(const float (&phi)[dim], float y) {
    float p_phi[dim];
    float denom = lambda_;
    for (uint32_t i = 0; i < dim; i++) {
      p_phi[i] = 0.0f;
      for (uint32_t j = 0; j < dim; j++) {
        p_phi[i] += p_[i][j] * phi[j];
      }
      denom += phi[i] * p_phi[i];
    }
    float innovation = y;
    for (uint32_t i = 0; i < dim; i++) {
      innovation -= phi[i] * theta_[i];
    }
    float k[dim];
    for (uint32_t i = 0; i < dim; i++) {
      k[i] = p_phi[i] / denom;
      theta_[i] += k[i] * innovation;
    }
    float a[dim][dim];  // I - K*phi'
    for (uint32_t i = 0; i < dim; i++) {
      for (uint32_t j = 0; j < dim; j++) {
        a[i][j] = (i == j ? 1.0f : 0.0f) - k[i] * phi[j];
      }
    }
    float ap[dim][dim];  // A*P
    for (uint32_t i = 0; i < dim; i++) {
      for (uint32_t j = 0; j < dim; j++) {
        ap[i][j] = 0.0f;
        for (uint32_t l = 0; l < dim; l++) {
          ap[i][j] += a[i][l] * p_[l][j];
        }
      }
    }
    float new_p[dim][dim];
    for (uint32_t i = 0; i < dim; i++) {
      for (uint32_t j = 0; j < dim; j++) {
        new_p[i][j] = k[i] * k[j];
        for (uint32_t l = 0; l < dim; l++) {
          new_p[i][j] += ap[i][l] * a[j][l];
        }
        new_p[i][j] /= lambda_;
      }
    }
    for (uint32_t i = 0; i < dim; i++) {
      for (uint32_t j = 0; j < dim; j++) {
        p_[i][j] = 0.5f * (new_p[i][j] + new_p[j][i]);
      }
    }
  }

  /// @brief 参数投影（物理界限），在 Update 后调用。下限保证正性，
  /// 上限阻断发散通道——实车实测缺失上限时毛刺可把 theta 推至 1e19。
  void Project(uint32_t index, float minimum, float maximum) {
    if (theta_[index] < minimum) {
      theta_[index] = minimum;
    }
    if (theta_[index] > maximum) {
      theta_[index] = maximum;
    }
  }

  /// @brief theta 与 P 全部有限（NaN/inf 看门狗）。
  [[nodiscard]] bool IsFinite() const {
    for (uint32_t i = 0; i < dim; i++) {
      if (!std::isfinite(theta_[i])) {
        return false;
      }
      for (uint32_t j = 0; j < dim; j++) {
        if (!std::isfinite(p_[i][j])) {
          return false;
        }
      }
    }
    return true;
  }

  [[nodiscard]] float Theta(uint32_t index) const { return theta_[index]; }

 private:
  float p_[dim][dim]{};
  float theta_[dim]{};
  float lambda_ = 1.0f;
};

class SystemIdentify final {
 public:
  /**
   * @brief 辨识调参与功能开关。
   *
   * @note 默认值**必须与 `Gimbal.hpp` 里 `system_identify` 的 manifest
   * 默认值逐字段一致**（`tests/gimbal_config_order_regression.py`
   * 会比对，分叉即失败）。实车值在 `User/RobotConfig/` 按位置覆盖。
   * @note 尾部为多个 float 字段连排：yaml 键序/字段序由回归脚本守护，
   *       新增字段必须同步三处（本结构体、manifest、回归脚本期望）。
   */
  struct Config {
    bool enabled{false};
    IdentifyAxis axis_select{IdentifyAxis::BOTH};
    float arming_delay_s{3.0f};
    float step_torque_yaw_nm{0.5f};
    float step_torque_pit_nm{0.2f};
    float step_half_period_s{0.5f};
    uint32_t step_cycles{10};
    float settle_time_s{1.0f};
    float lpf_cutoff_hz{50.0f};
    float alpha_gate_rad_s2{1.0f};
    float rls_delta{1000.0f};
    float rls_lambda{1.0f};
    float coulomb_tanh_scale{0.1f};
    float omega_abort_yaw{8.0f};
    float omega_abort_pit{4.0f};
    float pit_angle_margin_rad{0.1f};
    float axis_timeout_s{30.0f};
  };

  // xrobot 按声明顺序位置聚合 Config，这三条契约守护该初始化方式。
  static_assert(std::is_aggregate_v<Config>);
  static_assert(std::is_trivially_copyable_v<Config>);
  static_assert(std::is_standard_layout_v<Config>);

  /// @brief 每周期传感输入（云台约定符号，与 Gimbal 消费陀螺/欧拉角一致）。
  struct Sensors {
    float yaw_omega_rad_s = 0.0f;
    float pit_omega_rad_s = 0.0f;
    float pit_theta_rad = 0.0f;  ///< 重力回归项 sin(theta) 用
    bool yaw_motor_online = false;
    bool pit_motor_online = false;
    bool rc_online = false;
  };

  /// @brief 单轴辨识结果（收敛后锁存，Ozone 只读查看）。
  struct AxisResult {
    float j = 0.0f;
    float b = 0.0f;
    float tau_c = 0.0f;
    float mgl = 0.0f;  ///< 仅 pitch 有效
    float residual_var = 0.0f;
    float signal_var = 0.0f;
    float residual_ratio = 0.0f;  ///< 归一化残差（等效 1 - R^2）
    uint32_t update_count = 0;
    bool converged = false;
  };

  /**
   * @brief 用锁存的配置构造辨识器。
   * @note 不提供默认构造：与 YawSmc/YawLqrEso 同范式，完美转发 + requires
   *       防止劫持隐式拷贝构造。
   */
  template <typename ConfigType>
    requires std::same_as<std::remove_cvref_t<ConfigType>, Config>
  explicit SystemIdentify(ConfigType&& config)
      : config_(std::forward<ConfigType>(config)),
        rls_yaw_(config_.rls_delta, config_.rls_lambda),
        rls_pit_(config_.rls_delta, config_.rls_lambda) {}

  /// @brief 只读配置访问器（诊断与主机测试用）。
  [[nodiscard]] const Config& GetConfig() const noexcept { return config_; }

  static bool ValidateConfig(const Config& config) {
    if (!std::isfinite(config.arming_delay_s) || config.arming_delay_s < 0.0f ||
        !std::isfinite(config.step_torque_yaw_nm) ||
        config.step_torque_yaw_nm <= 0.0f ||
        !std::isfinite(config.step_torque_pit_nm) ||
        config.step_torque_pit_nm <= 0.0f ||
        !std::isfinite(config.step_half_period_s) ||
        config.step_half_period_s <= 0.0f || config.step_cycles == 0U ||
        !std::isfinite(config.settle_time_s) || config.settle_time_s < 0.0f ||
        !std::isfinite(config.lpf_cutoff_hz) || config.lpf_cutoff_hz <= 0.0f ||
        !std::isfinite(config.alpha_gate_rad_s2) ||
        config.alpha_gate_rad_s2 < 0.0f || !std::isfinite(config.rls_delta) ||
        config.rls_delta <= 0.0f || !std::isfinite(config.rls_lambda) ||
        config.rls_lambda <= 0.0f || config.rls_lambda > 1.0f ||
        !std::isfinite(config.coulomb_tanh_scale) ||
        config.coulomb_tanh_scale <= 0.0f ||
        !std::isfinite(config.omega_abort_yaw) ||
        config.omega_abort_yaw <= 0.0f ||
        !std::isfinite(config.omega_abort_pit) ||
        config.omega_abort_pit <= 0.0f ||
        !std::isfinite(config.pit_angle_margin_rad) ||
        config.pit_angle_margin_rad < 0.0f ||
        !std::isfinite(config.axis_timeout_s) ||
        config.axis_timeout_s <= 0.0f) {
      return false;
    }
    return true;
  }

  /// @brief 进入 ARMING（由 Gimbal 在进入 SET_MODE_IDENTIFY 时调用一次）。
  void Begin() {
    state_ = IdentifyState::ARMING;
    state_time_s_ = 0.0f;
    abort_reason_ = IdentifyAbort::NONE;
  }

  /// @brief 外部中止（辨识期间收到任何外部 GimbalEvent）。
  void Abort(IdentifyAbort reason) {
    if (!IsActive()) {
      return;
    }
    abort_reason_ = reason;
    state_ = IdentifyState::ABORTED;
  }

  [[nodiscard]] bool IsActive() const {
    return state_ != IdentifyState::IDLE && state_ != IdentifyState::DONE &&
           state_ != IdentifyState::ABORTED;
  }

  [[nodiscard]] IdentifyState State() const { return state_; }
  [[nodiscard]] IdentifyAbort AbortReason() const { return abort_reason_; }
  [[nodiscard]] const AxisResult& YawResult() const { return yaw_result_; }
  [[nodiscard]] const AxisResult& PitResult() const { return pit_result_; }

  /// @brief 当前激励生效轴（NONE 表示双轴 Relax）。
  enum class ActiveAxis : uint8_t { NONE, YAW, PITCH };

  [[nodiscard]] ActiveAxis GetActiveAxis() const {
    if (state_ == IdentifyState::YAW_EXCITE) {
      return ActiveAxis::YAW;
    }
    if (state_ == IdentifyState::PITCH_EXCITE) {
      return ActiveAxis::PITCH;
    }
    return ActiveAxis::NONE;
  }

  /**
   * @brief 每周期步进，返回生效轴的力矩指令（N*m）。
   *
   * 数据流：tau_raw（方波）与 omega 经同一一阶低通（相位对齐），
   * alpha 由滤波后 omega 差分再过同一低通；|alpha| 超门限时 RLS 更新，
   * 否则完全冻结（P/theta 不动）。
   */
  float Update(const Sensors& sensors, float dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0f) {
      return 0.0f;
    }
    // dt 钳制到控制环正常带内：墙钟实测 dt 在调度追赶时可能远小于 1ms，
    // alpha = 差分/dt 会按比例爆炸并不可逆污染 RLS（实车实测残余溢出 inf）。
    dt_s = std::clamp(dt_s, MIN_DT_S, MAX_DT_S);
    state_time_s_ += dt_s;

    switch (state_) {
      case IdentifyState::ARMING:
        return StepArming(sensors, dt_s);
      case IdentifyState::YAW_EXCITE:
        return StepExcite(sensors, dt_s, true);
      case IdentifyState::YAW_SETTLE:
        return StepSettle(dt_s, true);
      case IdentifyState::PITCH_EXCITE:
        return StepExcite(sensors, dt_s, false);
      case IdentifyState::PITCH_SETTLE:
        return StepSettle(dt_s, false);
      case IdentifyState::IDLE:
      case IdentifyState::DONE:
      case IdentifyState::ABORTED:
      default:
        return 0.0f;
    }
  }

 private:
  static constexpr float MIN_J_KG_M2 = 1e-5f;
  static constexpr float MAX_J_KG_M2 = 1.0f;
  static constexpr float MAX_B_NMS_RAD = 5.0f;
  static constexpr float MAX_TAU_C_NM = 2.0f;
  static constexpr float MAX_MGL_NM = 5.0f;
  static constexpr float MIN_DT_S = 0.0005f;
  static constexpr float MAX_DT_S = 0.002f;
  // innovation 野值上限：激励幅值 <= 0.5 N*m，合理模型误差不可能超此量级；
  // 超过即判为传感毛刺/dt 异常并跳过本次更新，阻断发散通道。
  static constexpr float INNOVATION_LIMIT_NM = 2.0f;
  static constexpr float CONVERGED_RESIDUAL_RATIO = 0.1f;

  const Config config_;
  RlsIdent<3> rls_yaw_;  ///< [J, B, tau_c]
  RlsIdent<4> rls_pit_;  ///< [J, B, tau_c, mgl]

  IdentifyState state_ = IdentifyState::IDLE;
  IdentifyAbort abort_reason_ = IdentifyAbort::NONE;
  float state_time_s_ = 0.0f;

  // 滤波链状态（仅激励期使用，进入 EXCITE 时播种）。
  // omega/tau 两级级联 LPF，alpha 取二级滤波后 omega 的差分：
  // 全部回归量共享相同的二阶滤波响应（相位对齐），且 alpha 噪声
  // 较单级方案抑制一个 LPF 增益量级。
  float omega_f1_ = 0.0f;
  float omega_f2_ = 0.0f;
  float tau_f1_ = 0.0f;
  float tau_f2_ = 0.0f;
  float coulomb_f1_ = 0.0f;
  float coulomb_f2_ = 0.0f;
  float gravity_f1_ = 0.0f;
  float gravity_f2_ = 0.0f;
  float alpha_f_ = 0.0f;
  float omega_f2_prev_ = 0.0f;
  float tau_f2_prev_ = 0.0f;

  // 残差统计（门控更新时累计）。
  float residual_sq_sum_ = 0.0f;
  float signal_sq_sum_ = 0.0f;
  uint32_t update_count_ = 0;

  AxisResult yaw_result_{};
  AxisResult pit_result_{};

  float LpfAlpha(float dt_s) const {
    const float RC =
        1.0f / (static_cast<float>(LibXR::TWO_PI) * config_.lpf_cutoff_hz);
    return dt_s / (dt_s + RC);
  }

  float StepArming(const Sensors& sensors, float /*dt_s*/) {
    if (!sensors.rc_online) {
      state_time_s_ = 0.0f;  // RC 离线期间持续等待，上线后重新倒计时
      return 0.0f;
    }
    if (state_time_s_ >= config_.arming_delay_s) {
      if (config_.axis_select == IdentifyAxis::PITCH_ONLY) {
        EnterExcite(false);
      } else {
        EnterExcite(true);  // YAW_ONLY / BOTH 均 yaw 先行
      }
    }
    return 0.0f;
  }

  void EnterExcite(bool yaw_axis) {
    state_ = yaw_axis ? IdentifyState::YAW_EXCITE : IdentifyState::PITCH_EXCITE;
    state_time_s_ = 0.0f;
    omega_f1_ = 0.0f;
    omega_f2_ = 0.0f;
    tau_f1_ = 0.0f;
    tau_f2_ = 0.0f;
    coulomb_f1_ = 0.0f;
    coulomb_f2_ = 0.0f;
    gravity_f1_ = 0.0f;
    gravity_f2_ = 0.0f;
    alpha_f_ = 0.0f;
    omega_f2_prev_ = 0.0f;
    tau_f2_prev_ = 0.0f;
    residual_sq_sum_ = 0.0f;
    signal_sq_sum_ = 0.0f;
    update_count_ = 0;
    if (yaw_axis) {
      rls_yaw_.Reset(config_.rls_delta, config_.rls_lambda);
    } else {
      rls_pit_.Reset(config_.rls_delta, config_.rls_lambda);
    }
  }

  float StepExcite(const Sensors& sensors, float dt_s, bool yaw_axis) {
    const float OMEGA_ABORT =
        yaw_axis ? config_.omega_abort_yaw : config_.omega_abort_pit;
    const bool MOTOR_ONLINE =
        yaw_axis ? sensors.yaw_motor_online : sensors.pit_motor_online;
    const float OMEGA_RAW =
        yaw_axis ? sensors.yaw_omega_rad_s : sensors.pit_omega_rad_s;

    // 传感输入有限性检查：陀螺/欧拉角毛刺（NaN/inf）会经差分放大进入
    // alpha 并永久污染 RLS；本周期跳过全部滤波与更新，激励力矩照常输出。
    if (!std::isfinite(OMEGA_RAW) || !std::isfinite(sensors.pit_theta_rad)) {
      const float TAU_SKIP =
          yaw_axis ? config_.step_torque_yaw_nm : config_.step_torque_pit_nm;
      const uint32_t HALF_SKIP =
          static_cast<uint32_t>(state_time_s_ / config_.step_half_period_s);
      const float AMP_SKIP = (HALF_SKIP / 2U % 2U == 0U) ? 1.0f : 0.6f;
      return ((HALF_SKIP % 2U == 0U) ? TAU_SKIP : -TAU_SKIP) * AMP_SKIP;
    }

    if (!MOTOR_ONLINE) {
      Abort(IdentifyAbort::MOTOR_OFFLINE);
      return 0.0f;
    }
    if (std::fabs(OMEGA_RAW) > OMEGA_ABORT) {
      Abort(IdentifyAbort::OMEGA_LIMIT);
      return 0.0f;
    }
    if (state_time_s_ > config_.axis_timeout_s) {
      Abort(IdentifyAbort::AXIS_TIMEOUT);
      return 0.0f;
    }

    // 激励发生器：每半个周期换向，2*step_cycles 个半周期后完成。
    // 幅值按周期在 1.0x/0.6x 间交替：恒定幅值下稳态 omega 相同，导致
    // 回归量 [omega, tanh(omega/w0)] 近似共线（B 与 tau_c 不可分、
    // 含噪时 RLS 发散）；双幅值给出两个稳态工作点以解相关。
    const float TAU_0 =
        yaw_axis ? config_.step_torque_yaw_nm : config_.step_torque_pit_nm;
    const uint32_t HALF_INDEX =
        static_cast<uint32_t>(state_time_s_ / config_.step_half_period_s);
    if (HALF_INDEX >= 2U * config_.step_cycles) {
      state_ =
          yaw_axis ? IdentifyState::YAW_SETTLE : IdentifyState::PITCH_SETTLE;
      state_time_s_ = 0.0f;
      return 0.0f;
    }
    const float AMPLITUDE_SCALE = (HALF_INDEX / 2U % 2U == 0U) ? 1.0f : 0.6f;
    const float TAU_RAW =
        ((HALF_INDEX % 2U == 0U) ? TAU_0 : -TAU_0) * AMPLITUDE_SCALE;

    // 统一滤波链：omega/tau 两级级联 LPF，alpha 取二级滤波后 omega 的
    // 差分。LTI 系统 F(dω/dt)=d/dt[F(ω)]，三个回归量共享完全相同的
    // 二阶滤波响应（相位/幅值对齐），且 alpha 的测量噪声被二级滤波抑制。
    const float LPF_A = LpfAlpha(dt_s);
    omega_f1_ += LPF_A * (OMEGA_RAW - omega_f1_);
    omega_f2_prev_ = omega_f2_;
    omega_f2_ += LPF_A * (omega_f1_ - omega_f2_);
    // 观测 y 滞后一拍：本周期的 omega/alpha 响应的是上周期下发的力矩，
    // 回归必须用上一周期的滤波力矩，否则 B/tau_c 产生约 -10%/+5% 的
    // 系统性偏差（仿真实测：滞后一拍打后偏差消除到 0.1% 以内）。
    tau_f2_prev_ = tau_f2_;
    tau_f1_ += LPF_A * (TAU_RAW - tau_f1_);
    tau_f2_ += LPF_A * (tau_f1_ - tau_f2_);
    // 非线性回归量先对原始信号逐点计算再过同一二阶滤波（严格"滤波回归"：
    // 对象方程每一项都是真实信号经同一滤波器后的样子；若改为对滤波后
    // omega 取 tanh/sin，非线性环节与滤波不可交换，会引入系统性偏差）。
    const float COULOMB_RAW = std::tanh(OMEGA_RAW / config_.coulomb_tanh_scale);
    coulomb_f1_ += LPF_A * (COULOMB_RAW - coulomb_f1_);
    coulomb_f2_ += LPF_A * (coulomb_f1_ - coulomb_f2_);
    const float GRAVITY_RAW = std::sin(sensors.pit_theta_rad);
    gravity_f1_ += LPF_A * (GRAVITY_RAW - gravity_f1_);
    gravity_f2_ += LPF_A * (gravity_f1_ - gravity_f2_);
    alpha_f_ = (omega_f2_ - omega_f2_prev_) / dt_s;

    // 双门控：
    // ① |alpha| 超阈值（剔除低信息量稳态样本，防协方差 windup）；
    // ② |omega| 超 tanh 饱和点（5*w0）：低速段 tanh(omega/w0)≈omega/w0 与
    //    omega 回归量近似共线（B/tau_c 不可分），此时更新会让 RLS 沿零空间
    //    方向随机游走直至发散（仿真实测 theta 达 1e6 量级），必须冻结。
    const float OMEGA_GATE = 5.0f * config_.coulomb_tanh_scale;
    if (std::fabs(alpha_f_) > config_.alpha_gate_rad_s2 &&
        std::fabs(omega_f2_) > OMEGA_GATE) {
      if (yaw_axis) {
        const float PHI[3] = {alpha_f_, omega_f2_, coulomb_f2_};
        const float INNOVATION = tau_f2_prev_ - PHI[0] * rls_yaw_.Theta(0) -
                                 PHI[1] * rls_yaw_.Theta(1) -
                                 PHI[2] * rls_yaw_.Theta(2);
        // 野值剔除：超上限的 innovation 判为毛刺，跳过更新与统计。
        if (std::fabs(INNOVATION) <= INNOVATION_LIMIT_NM) {
          rls_yaw_.Update(PHI, tau_f2_prev_);
          rls_yaw_.Project(0, MIN_J_KG_M2, MAX_J_KG_M2);
          rls_yaw_.Project(1, 0.0f, MAX_B_NMS_RAD);
          rls_yaw_.Project(2, -MAX_TAU_C_NM, MAX_TAU_C_NM);
          AccumulateResidual(INNOVATION, tau_f2_prev_);
        }
        // NaN/inf 看门狗：任何数值异常立即中止，垃圾结果不得锁存。
        if (!rls_yaw_.IsFinite()) {
          Abort(IdentifyAbort::NUMERICAL_DIVERGENCE);
          return 0.0f;
        }
      } else {
        const float PHI[4] = {alpha_f_, omega_f2_, coulomb_f2_, gravity_f2_};
        const float INNOVATION = tau_f2_prev_ - PHI[0] * rls_pit_.Theta(0) -
                                 PHI[1] * rls_pit_.Theta(1) -
                                 PHI[2] * rls_pit_.Theta(2) -
                                 PHI[3] * rls_pit_.Theta(3);
        if (std::fabs(INNOVATION) <= INNOVATION_LIMIT_NM) {
          rls_pit_.Update(PHI, tau_f2_prev_);
          rls_pit_.Project(0, MIN_J_KG_M2, MAX_J_KG_M2);
          rls_pit_.Project(1, 0.0f, MAX_B_NMS_RAD);
          rls_pit_.Project(2, -MAX_TAU_C_NM, MAX_TAU_C_NM);
          rls_pit_.Project(3, -MAX_MGL_NM, MAX_MGL_NM);
          AccumulateResidual(INNOVATION, tau_f2_prev_);
        }
        if (!rls_pit_.IsFinite()) {
          Abort(IdentifyAbort::NUMERICAL_DIVERGENCE);
          return 0.0f;
        }
      }
    }
    return TAU_RAW;
  }

  void AccumulateResidual(float innovation, float signal) {
    residual_sq_sum_ += innovation * innovation;
    signal_sq_sum_ += signal * signal;
    update_count_++;
  }

  float StepSettle(float /*dt_s*/, bool yaw_axis) {
    if (state_time_s_ >= config_.settle_time_s) {
      LatchResult(yaw_axis);
      if (yaw_axis) {
        if (config_.axis_select == IdentifyAxis::BOTH) {
          EnterExcite(false);
        } else {
          state_ = IdentifyState::DONE;
        }
      } else {
        state_ = IdentifyState::DONE;
      }
    }
    return 0.0f;
  }

  void LatchResult(bool yaw_axis) {
    AxisResult& result = yaw_axis ? yaw_result_ : pit_result_;
    if (yaw_axis) {
      result.j = rls_yaw_.Theta(0);
      result.b = rls_yaw_.Theta(1);
      result.tau_c = rls_yaw_.Theta(2);
      result.mgl = 0.0f;
    } else {
      result.j = rls_pit_.Theta(0);
      result.b = rls_pit_.Theta(1);
      result.tau_c = rls_pit_.Theta(2);
      result.mgl = rls_pit_.Theta(3);
    }
    if (update_count_ > 0U) {
      result.residual_var = residual_sq_sum_ / update_count_;
      result.signal_var = signal_sq_sum_ / update_count_;
      result.residual_ratio = (result.signal_var > 0.0f)
                                  ? result.residual_var / result.signal_var
                                  : 0.0f;
    }
    result.update_count = update_count_;
    // 收敛判据硬化：残差比有限且低于阈值才算收敛——仅按更新次数会把
    // 发散后的垃圾值伪装成成功（实车实测 j=1.97/b=36 仍 converged=1）。
    result.converged = update_count_ > 0U &&
                       std::isfinite(result.residual_ratio) &&
                       result.residual_ratio < CONVERGED_RESIDUAL_RATIO &&
                       std::isfinite(result.j) && std::isfinite(result.b) &&
                       std::isfinite(result.tau_c) && std::isfinite(result.mgl);
  }
};

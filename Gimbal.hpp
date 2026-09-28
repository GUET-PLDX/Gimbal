#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_yaw_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_yaw_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 2.223
      cycle: true
  - pid_pit_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_pit_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_pitch: '@&motor_pit'
  - motor_yaw: '@&motor_yaw'
  - gimbal_param:
      pit_max_angle: 0.0
      pit_min_angle: 0.0
      pit_lc: 0.0
      pit_theta: 0.0
      yaw_k: 0.0
      j_pit: 0.0
      j_yaw: 0.0
      pit_zero: 0.0
      yaw_zero: 0.0
      patrol_pitch_amplitude_rad: 0.0
      patrol_pitch_angular_rate_rad_s: 0.0
      patrol_yaw_rate_rad_s: 0.0
      reverse_flag: false
      thread_priority: LibXR::Thread::Priority::MEDIUM
      rotor_ff_enabled: false
      yaw_manual_controller: YawManualController::PID
      yaw_ai_controller: YawAiController::LQR_ESO
  - yaw_lqr_eso:
      k_theta: 1.0
      k_omega: 1.0
      k_i: 0.2
      theta_integral_limit_rad_s: 0.5
      tau_coulomb_nm: 0.05
      coulomb_smooth_rad_s: 0.2
      eso_bandwidth_rad_s: 30.0
      eso_comp_gain: 1.0
      eso_comp_limit_nm: 0.3
      eso_omega_gate_rad_s: 5.0
      eso_alpha_gate_rad_s2: 50.0
      tau_bias_ki: 0.5
      tau_bias_limit_nm: 0.15
      tau_meas_lpf_alpha: 0.1
      theta_deadband_rad: 0.0
      torque_soft_limit_nm: 2.0
      torque_slew_rate_nm_s: 1000.0
      eso_enable: true
      eso_comp_enable: false
      coulomb_enable: false
      lqi_enable: false
      torque_bias_enable: false
      torque_slew_enable: true
  - yaw_smc:
      c: 20.0
      k: 120.0
      epsilon: 0.5
      q: 21.0
      p: 27.0
      error_deadband_rad: 0.0
      ftsmc_switch_rad: 0.017453292519943295
      sat_boundary: 1.0
      torque_soft_limit_nm: 2.0
      torque_min_nm: -2.223
      torque_max_nm: 2.223
      torque_slew_rate_nm_s: 1000.0
      ftsmc_enable: true
      torque_slew_enable: true
  - system_identify:
      enabled: false
      axis_select: IdentifyAxis::BOTH
      arming_delay_s: 3.0
      step_torque_yaw_nm: 0.5
      step_torque_pit_nm: 0.2
      step_half_period_s: 0.5
      step_cycles: 10
      settle_time_s: 1.0
      lpf_cutoff_hz: 50.0
      alpha_gate_rad_s2: 1.0
      rls_delta: 1000.0
      rls_lambda: 1.0
      coulomb_tanh_scale: 0.1
      omega_abort_yaw: 8.0
      omega_abort_pit: 4.0
      pit_angle_margin_rad: 0.1
      axis_timeout_s: 30.0
template_args: []
required_hardware: []
depends:
  - pldx/CMD
  - pldx/Motor
  - pldx/BMI088
  - pldx/DualBoard
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <utility>

#include "CMD.hpp"
#include "DualBoard.hpp"
#include "Motor.hpp"
#include "SystemIdentify.hpp"
#include "YawLqrEso.hpp"
#include "YawSmc.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

using Pldx::DualBoardControl::CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER;
using Pldx::DualBoardControl::CHASSIS_MOTION_STATE_TOPIC_NAME;
using Pldx::DualBoardControl::ChassisMotionMode;
using Pldx::DualBoardControl::ChassisMotionState;

enum class GimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
  SET_MODE_AUTOPATROL,
  SET_VISION_IDLE,
  SET_VISION_AUTO_AIM,
  SET_VISION_SMALL_BUFF,
  SET_VISION_BIG_BUFF,
  SET_MODE_IDENTIFY
};
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_RELAX) == 0U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_COMMON) == 1U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_AUTOPATROL) == 2U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_VISION_IDLE) == 3U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_VISION_AUTO_AIM) == 4U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_VISION_SMALL_BUFF) == 5U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_VISION_BIG_BUFF) == 6U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_IDENTIFY) == 7U);
enum class YawManualController : uint8_t { PID, SMC };
enum class YawAiController : uint8_t { LQR_ESO, SMC };
class Gimbal : public LibXR::Application {
 public:
  struct GimbalParam {
    float pit_max_angle = 0.0f;
    float pit_min_angle = 0.0f;
    float pit_lc = 0.0f;
    float pit_theta = 0.0f;
    float yaw_k = 0.0f;
    float j_pit = 0.0f;
    float j_yaw = 0.0f;
    float pit_zero = 0.0f;
    float yaw_zero = 0.0f;
    float patrol_pitch_amplitude_rad = 0.0f;
    float patrol_pitch_angular_rate_rad_s = 0.0f;
    float patrol_yaw_rate_rad_s = 0.0f;
    bool reverse_flag = false;
    LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM;
    bool rotor_ff_enabled = false;
    YawManualController yaw_manual_controller = YawManualController::PID;
    YawAiController yaw_ai_controller = YawAiController::LQR_ESO;
  };

  struct CycleFeedforward {
    float yaw_omega = 0.0f;
    float yaw_alpha = 0.0f;
    float yaw_angle = 0.0f;
    float pitch_omega = 0.0f;
    float pitch_alpha = 0.0f;
    float pitch_angle = 0.0f;
    bool ai_yaw_active = false;
  };

  /**
   * @brief 构造函数初始化数据成员
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param cmd 命令模块实例
   * @param task_stack_depth 任务堆栈深度
   * @param pid_yaw_angle Yaw轴角度环PID参数
   * @param pid_yaw_omega Yaw轴角速度环PID参数
   * @param pid_pit_angle Pitch轴角度环PID参数
   * @param pid_pit_omega Pitch轴角速度环PID参数
   * @param motor_pit Pitch轴电机指针
   * @param motor_yaw Yaw轴电机指针
   * @param pit_max_angle Pitch轴最大角度
   * @param pit_min_angle Pitch轴最小角度
   * @param pit_lc Pitch质心距离(m)(距离水平向上为+)*Pitch质心重力(N)
   * @param pit_theta Pitch质心与重力轴线夹角(rad 极性自己猜)
   * @param yaw_k Yaw轴阻力系数
   * @param j_pit Pitch轴转动惯量
   * @param j_yaw Yaw轴转动惯量
   * @param pit_zero Pitch轴零点
   * @param yaw_zero Yaw轴零点
   * @param patrol_pitch_amplitude_rad 巡逻Pitch幅度(rad)
   * @param patrol_pitch_angular_rate_rad_s 巡逻Pitch角频率(rad/s)
   * @param patrol_yaw_rate_rad_s 巡逻Yaw角速度(rad/s)
   * @param reverse_flag Pitch轴反转标志
   * @param rotor_ff_enabled
   *
   * 是否启用小陀螺模式Yaw轴角速度前馈
   * @param yaw_manual_controller 手动 Yaw 控制器选择，仅 PID 或 SMC
   * @param yaw_ai_controller AI Yaw 控制器选择，LQR/ESO 或 SMC
   * @param yaw_lqr_eso AI Yaw LQR/ESO参数
   * @param yaw_smc Yaw 滑模参数
   * @param system_identify 系统辨识配置（一次性标定工具，enabled
   * 时上电进入辨识）
   *
   * @note `yaw_lqr_eso` / `yaw_smc` / `system_identify` **故意不给默认实参**。
   * `xrobot_gen_main` 按位置 展开 `constructor_args`，YAML
   * 里漏掉某个键时它会**直接丢弃该实参而不是 回退到 manifest
   * 默认值**，于是其后所有实参整体左移（实测：省掉 `yaw_lqr_eso` 会把 `yaw_smc`
   * 的聚合值喂给 `yaw_lqr_eso`，而 `yaw_smc`
   *       落到默认值）。让最后一个形参没有默认值，就把"漏键"从静默错配变成
   *       **编译错误**。
   */
  Gimbal(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
         uint32_t task_stack_depth, LibXR::PID<float>::Param pid_yaw_angle,
         LibXR::PID<float>::Param pid_yaw_omega,
         LibXR::PID<float>::Param pid_pit_angle,
         LibXR::PID<float>::Param pid_pit_omega, Motor* motor_pit,
         Motor* motor_yaw, GimbalParam gimbal_param,
         YawLqrEso::Config yaw_lqr_eso, YawSmc::Config yaw_smc,
         SystemIdentify::Config system_identify)
      : cmd_(cmd),
        pid_yaw_angle_(pid_yaw_angle),
        pid_yaw_omega_(pid_yaw_omega),
        pid_pit_angle_(pid_pit_angle),
        pid_pit_omega_(pid_pit_omega),
        motor_yaw_(motor_yaw),
        motor_pit_(motor_pit),
        PARAM(gimbal_param),
        yaw_lqr_eso_(
            CheckedLqrEsoConfig(std::move(yaw_lqr_eso), gimbal_param.j_yaw,
                                pid_yaw_omega_.OutLimit(), gimbal_param.yaw_k)),
        yaw_smc_(CheckedSmcConfig(std::move(yaw_smc), gimbal_param.j_yaw)),
        system_identify_(CheckedSysIdConfig(std::move(system_identify))) {
    UNUSED(app);
    InitializeTopics();
    ChassisMotionStateTopic();
    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   PARAM.thread_priority);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->SetMode(static_cast<GimbalEvent>(event_id));
        },
        this);
    cmd_.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_COMMON),
                           callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOPATROL), callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_VISION_IDLE),
                           callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_VISION_AUTO_AIM), callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_VISION_SMALL_BUFF), callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_VISION_BIG_BUFF), callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_IDENTIFY), callback);
  };

  /**
   * @brief 线程函数
   *
   * @param gimbal Gimbal实例指针
   */
  static void ThreadFunc(Gimbal* gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber("gimbal_cmd");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        "gimbal_euler");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        "gimbal_gyro");
    LibXR::Topic::ASyncSubscriber<ChassisMotionState>
        chassis_motion_state_suber(ChassisMotionStateTopic());
    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();
    chassis_motion_state_suber.StartWaiting();

    gimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    while (true) {
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (euler_suber.Available()) {
        auto euler_sample = euler_suber.GetData();
        euler_sample.Pitch() *= -1.0f;
        gimbal->euler_ = euler_sample;
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        auto gyro_sample = gyro_suber.GetData();
        gyro_sample.y() *= -1.0f;
        gimbal->gyro_data_ = gyro_sample;
        gyro_suber.StartWaiting();
      }
      if (chassis_motion_state_suber.Available()) {
        gimbal->chassis_motion_state_ = chassis_motion_state_suber.GetData();
        chassis_motion_state_suber.StartWaiting();
      }

      gimbal->Update();
      const auto FEEDFORWARD = gimbal->ParseCMD();
      gimbal->Control(FEEDFORWARD);
      LibXR::Thread::SleepUntil(last_time, 1);
    }
  };

  /**
   * @brief 更新电机反馈及状态
   */
  void Update() {
    // 上电一次性钩子：system_identify.enabled 时自动进入辨识模式。
    // 辨识完成后锁死 RELAX（见 SetMode），须以 enabled:false 重启恢复正常使用。
    if (!identify_boot_checked_) {
      identify_boot_checked_ = true;
      if (system_identify_.GetConfig().enabled &&
          current_mode_ == GimbalEvent::SET_MODE_RELAX) {
        SetMode(GimbalEvent::SET_MODE_IDENTIFY);
      }
    }
    motor_yaw_->Update();
    motor_pit_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_pit_feedback_ = motor_pit_->GetFeedback();

    const auto NOW = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (NOW - this->last_online_time_).ToSecondf();
    this->last_online_time_ = NOW;

    float yaw_encoder_relative_angle =
        motor_yaw_feedback_.abs_angle - PARAM.yaw_zero;
    float pitch_encoder_relative_angle =
        motor_pit_feedback_.abs_angle - PARAM.pit_zero;
    YawAngleTopic().Publish(yaw_encoder_relative_angle);
    PitchAngleTopic().Publish(pitch_encoder_relative_angle);
    YawOmegaTopic().Publish(gyro_data_.z());
    PitchOmegaTopic().Publish(gyro_data_.y());
    uint8_t mode = static_cast<uint8_t>(current_mode_);
    ModeTopic().Publish(mode);
  }

  /**
   * @brief 解析云台控制命令
   */
  CycleFeedforward ParseCMD() {
    CycleFeedforward feedforward{};
    const auto CTRL_MODE = cmd_.GetCtrlMode();
    const bool AI_GIMBAL_ACTIVE = cmd_.GetAIGimbalStatus();
    const bool OPERATOR_CONTROL = CTRL_MODE == CMD::Mode::CMD_OP_CTRL;
    const bool AUTOPATROL = current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL;
    const bool VISION_MODE =
        current_mode_ == GimbalEvent::SET_VISION_AUTO_AIM ||
        current_mode_ == GimbalEvent::SET_VISION_SMALL_BUFF ||
        current_mode_ == GimbalEvent::SET_VISION_BIG_BUFF;
    const bool AI_YAW_ACTIVE = CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL &&
                               AI_GIMBAL_ACTIVE && VISION_MODE;
    feedforward.ai_yaw_active = AI_YAW_ACTIVE;

    if (AI_YAW_ACTIVE) {
      feedforward.pitch_angle = cmd_data_.pit;
      feedforward.pitch_omega = cmd_data_.pit_dot;
      feedforward.pitch_alpha = cmd_data_.pit_ddot;
      feedforward.yaw_angle = cmd_data_.yaw;
      feedforward.yaw_omega = cmd_data_.yaw_dot;
      feedforward.yaw_alpha = cmd_data_.yaw_ddot;
      return feedforward;
    }

    if (!OPERATOR_CONTROL && AUTOPATROL) {
      const float ELAPSED_S =
          static_cast<float>(
              (LibXR::Timebase::GetMilliseconds() - patrol_.start_time)
                  .ToMillisecond()) /
          1000.0f;
      constexpr float TWO_OVER_PI = 0.6366197723675814f;
      feedforward.pitch_angle =
          patrol_.pitch_center_rad +
          PARAM.patrol_pitch_amplitude_rad * TWO_OVER_PI *
              std::asin(
                  std::sin(PARAM.patrol_pitch_angular_rate_rad_s * ELAPSED_S));
      feedforward.yaw_angle =
          patrol_.yaw_origin_rad + PARAM.patrol_yaw_rate_rad_s * ELAPSED_S;
      feedforward.yaw_omega = PARAM.patrol_yaw_rate_rad_s;
      return feedforward;
    }

    feedforward.pitch_angle = cmd_data_.pit;
    feedforward.pitch_omega = cmd_data_.pit_dot;
    feedforward.pitch_alpha = cmd_data_.pit_ddot;
    feedforward.yaw_angle = cmd_data_.yaw;
    feedforward.yaw_omega = cmd_data_.yaw_dot;
    feedforward.yaw_alpha = cmd_data_.yaw_ddot;
    return feedforward;
  }

  /**
   * @brief 云台控制计算与输出
   */
  void Control(const CycleFeedforward& feedforward) {
    float pit_output = 0.0f;
    float yaw_output = 0.0f;

    if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
      SubmitRelaxOutput();
      return;
    }

    if (current_mode_ == GimbalEvent::SET_MODE_IDENTIFY) {
      ControlIdentify();
      return;
    }

    CycleFeedforward command = feedforward;
    PitchLimit(command.pitch_angle, euler_.Pitch(),
               motor_pit_feedback_.abs_angle, PARAM.pit_max_angle,
               PARAM.pit_min_angle, PARAM.reverse_flag ? 1.0f : -1.0f);
    const bool PATROL_PITCH =
        !feedforward.ai_yaw_active &&
        current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL &&
        cmd_.GetCtrlMode() != CMD::Mode::CMD_OP_CTRL;
    if (!PATROL_PITCH) {
      cmd_data_.pit = command.pitch_angle;
      cmd_.SetGimbalSetpoint(cmd_data_.yaw, command.pitch_angle);
    }
    Solve(command, pit_output, yaw_output);

    auto yaw_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = yaw_output});
    auto pit_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = pit_output});

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 and fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_pit_, motor_pit_feedback_, pit_motor_cmd);
    ControlYawMotor(yaw_motor_cmd);
  }

  void OnMonitor() override {}

  LibXR::Event& GetEvent() { return gimbal_event_; }

 private:
  CMD& cmd_;
  LibXR::PID<float> pid_yaw_angle_;
  LibXR::PID<float> pid_yaw_omega_;
  LibXR::PID<float> pid_pit_angle_;
  LibXR::PID<float> pid_pit_omega_;
  Motor* motor_yaw_;
  Motor* motor_pit_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pit_feedback_;

  CMD::GimbalCMD cmd_data_;
  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  const GimbalParam PARAM;

  /// 自动巡逻会话状态：进入 SET_MODE_AUTOPATROL
  /// 时一次性锁存起点，巡逻期间只读。
  struct PatrolState {
    float pitch_center_rad = 0.0f;
    float yaw_origin_rad = 0.0f;
    LibXR::MillisecondTimestamp start_time = 0.0f;
  };

  PatrolState patrol_{};
  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  YawLqrEso yaw_lqr_eso_;
  YawSmc yaw_smc_;
  SystemIdentify system_identify_;
  bool identify_boot_checked_ = false;
  bool identify_relax_lock_ = false;
  ChassisMotionState chassis_motion_state_{};
  LibXR::Thread thread_;

  /**
   * @brief 校验并透传 AI Yaw LQR/ESO 配置；非法配置在构造期直接挂起。
   * @note 校验放在初始化列表而不是构造函数体：① 函数体开头
   *       `UNUSED(app); InitializeTopics(); ChassisMotionStateTopic();
   *        thread_.Create(` 的连续片段被 tests/gimbal_core_static_regression.sh
   *       钉住，插不进语句；② 校验必须先于控制线程启动。
   * @note REQUIRE 只能出现在本文件：控制器头里调用会让主机测试链接阶段
   *       undefined reference（那些测试不链接 libxr_fatal_error）。
   */
  static YawLqrEso::Config CheckedLqrEsoConfig(YawLqrEso::Config config,
                                               float j_kg_m2,
                                               float torque_limit_nm,
                                               float b_nms_rad) {
    REQUIRE(
        YawLqrEso::ValidateConfig(config, j_kg_m2, torque_limit_nm, b_nms_rad));
    return config;
  }

  /// @brief 校验并透传 Yaw 滑模配置；非法配置在构造期直接挂起。
  static YawSmc::Config CheckedSmcConfig(YawSmc::Config config, float j_kg_m2) {
    REQUIRE(YawSmc::ValidateConfig(config, j_kg_m2));
    return config;
  }

  /// @brief 校验并透传系统辨识配置；非法配置在构造期直接挂起。
  static SystemIdentify::Config CheckedSysIdConfig(
      SystemIdentify::Config config) {
    REQUIRE(SystemIdentify::ValidateConfig(config));
    return config;
  }

  /*----------工具函数--------------------------------*/
  /**
   * @brief Pitch轴角度限位
   *
   * @param target_pit 目标Pitch角度
   * @param now_eulr_angle 当前Pitch欧拉角
   * @param now_motor_angle 当前Pitch电机角度
   * @param motor_max 电机最大角度
   * @param motor_min 电机最小角度
   * @param sign 方向符号
   */
  void PitchLimit(float& target_pit, float now_eulr_angle,
                  float now_motor_angle, float motor_max, float motor_min,
                  float sign) {
    if ((motor_max == 0.0f) && (motor_min == 0.0f)) {
      return;
    };

    LibXR::CycleValue<float> cycle_motor_min(motor_min);
    LibXR::CycleValue<float> cycle_motor_max(motor_max);

    float diff_min = cycle_motor_min - now_motor_angle;
    float diff_max = cycle_motor_max - now_motor_angle;
    float pitch_bound_0 = now_eulr_angle + diff_min / sign;
    float pitch_bound_1 = now_eulr_angle + diff_max / sign;

    float upper_bound = std::max(pitch_bound_0, pitch_bound_1);
    float lower_bound = std::min(pitch_bound_0, pitch_bound_1);
    target_pit = std::clamp(target_pit, lower_bound, upper_bound);
  }

  void SubmitRelaxOutput() {
    pid_pit_omega_.SetFeedForward(0.0f);
    pid_yaw_omega_.SetFeedForward(0.0f);
    motor_yaw_->Relax();
    motor_pit_->Relax();
  }

  /**
   * @brief 系统辨识模式控制输出（开环力矩双向阶跃 + RLS 在线辨识 J/B）。
   *
   * 生效轴输出激励力矩，非生效轴 Relax；pitch 角度限值取 GimbalParam
   * 机械限位内缩 `pit_angle_margin_rad` 安全边距，越界立即中止。
   * DONE/ABORTED 后锁死 RELAX（`identify_relax_lock_`），直到以
   * `system_identify.enabled: false` 重启。
   */
  void ControlIdentify() {
    if (system_identify_.GetActiveAxis() == SystemIdentify::ActiveAxis::PITCH) {
      const float MARGIN = system_identify_.GetConfig().pit_angle_margin_rad;
      const float PIT_ANGLE = motor_pit_feedback_.abs_angle;
      if (PIT_ANGLE < PARAM.pit_min_angle + MARGIN ||
          PIT_ANGLE > PARAM.pit_max_angle - MARGIN) {
        system_identify_.Abort(IdentifyAbort::PITCH_ANGLE_LIMIT);
      }
    }

    const SystemIdentify::Sensors SENSORS{
        .yaw_omega_rad_s = gyro_data_.z(),
        .pit_omega_rad_s = gyro_data_.y(),
        .pit_theta_rad = euler_.Pitch(),
        .yaw_motor_online = motor_yaw_->IsOnline(),
        .pit_motor_online = motor_pit_->IsOnline(),
        .rc_online = cmd_.Online(),
    };
    const float TAU = system_identify_.Update(SENSORS, dt_);
    const auto ACTIVE = system_identify_.GetActiveAxis();

    if (ACTIVE == SystemIdentify::ActiveAxis::YAW) {
      motor_pit_->Relax();
      ControlYawMotor(Motor::MotorCmd(
          {.mode = Motor::ControlMode::MODE_TORQUE, .torque = TAU}));
    } else if (ACTIVE == SystemIdentify::ActiveAxis::PITCH) {
      motor_yaw_->Relax();
      if (motor_pit_feedback_.state == 0) {
        motor_pit_->Enable();
      } else if (motor_pit_feedback_.state != 1) {
        motor_pit_->ClearError();
      } else {
        motor_pit_->Control(Motor::MotorCmd(
            {.mode = Motor::ControlMode::MODE_TORQUE, .torque = TAU}));
      }
    } else {
      SubmitRelaxOutput();
    }

    if (system_identify_.State() == IdentifyState::DONE ||
        system_identify_.State() == IdentifyState::ABORTED) {
      identify_relax_lock_ = true;
      SetMode(GimbalEvent::SET_MODE_RELAX);
    }
  }

  void ControlYawMotor(const Motor::MotorCmd& command) {
    if (motor_yaw_feedback_.state == 0) {
      motor_yaw_->Enable();
    } else if (motor_yaw_feedback_.state != 1) {
      motor_yaw_->ClearError();
    } else {
      motor_yaw_->Control(command);
      yaw_lqr_eso_.CommitAppliedTorque(command.torque);
      yaw_smc_.CommitAppliedTorque(command.torque);
    }
  }

  /**
   * @brief 解算PID控制输出
   */
  void Solve(const CycleFeedforward& feedforward, float& pit_output,
             float& yaw_output) {
    const float PIT_ERROR = feedforward.pitch_angle - euler_.Pitch();
    const float LAST_PIT_ANGLE_LOOP_OMEGA = pid_pit_angle_.LastOutput();
    const float PIT_ANGLE_LOOP_OMEGA =
        pid_pit_angle_.Calculate(PIT_ERROR, 0.0f, dt_);
    const float TARGET_PIT_OMEGA =
        PIT_ANGLE_LOOP_OMEGA + feedforward.pitch_omega;
    const float PIT_ALPHA =
        (PIT_ANGLE_LOOP_OMEGA - LAST_PIT_ANGLE_LOOP_OMEGA) / dt_ +
        feedforward.pitch_alpha;
    const float PITCH_FEEDFORWARD =
        PARAM.j_pit * PIT_ALPHA -
        PARAM.pit_lc * sinf(euler_.Pitch() + PARAM.pit_theta);
    pid_pit_omega_.SetFeedForward(PITCH_FEEDFORWARD);
    pit_output =
        pid_pit_omega_.Calculate(TARGET_PIT_OMEGA, gyro_data_.y(), dt_);

    if (feedforward.ai_yaw_active) {
      SolveAiYaw(yaw_output);
    } else if (PARAM.yaw_manual_controller == YawManualController::SMC) {
      SolveManualYawSmc(feedforward, yaw_output);
    } else {
      SolvePidYaw(feedforward, yaw_output);
    }
  }

  void SolvePidYaw(const CycleFeedforward& feedforward, float& yaw_output) {
    const float YAW_ERROR =
        LibXR::CycleValue<float>(feedforward.yaw_angle) - euler_.Yaw();
    const float LAST_YAW_ANGLE_LOOP_OMEGA = pid_yaw_angle_.LastOutput();
    const float YAW_ANGLE_LOOP_OMEGA =
        pid_yaw_angle_.Calculate(YAW_ERROR, 0.0f, dt_);
    const float TARGET_YAW_OMEGA = YAW_ANGLE_LOOP_OMEGA + feedforward.yaw_omega;
    const float YAW_ALPHA =
        (YAW_ANGLE_LOOP_OMEGA - LAST_YAW_ANGLE_LOOP_OMEGA) / dt_ +
        feedforward.yaw_alpha;
    const bool ROTOR_FF_ACTIVE =
        PARAM.rotor_ff_enabled && chassis_motion_state_.online &&
        chassis_motion_state_.yaw_rate_valid &&
        chassis_motion_state_.mode == ChassisMotionMode::ROTOR;
    const float YAW_MOTOR_OMEGA_REF =
        ROTOR_FF_ACTIVE
            ? TARGET_YAW_OMEGA - chassis_motion_state_.yaw_rate_rad_s
            : TARGET_YAW_OMEGA;
    const float YAW_FEEDFORWARD =
        PARAM.j_yaw * YAW_ALPHA + PARAM.yaw_k * YAW_MOTOR_OMEGA_REF;
    pid_yaw_omega_.SetFeedForward(YAW_FEEDFORWARD);
    yaw_output =
        pid_yaw_omega_.Calculate(TARGET_YAW_OMEGA, gyro_data_.z(), dt_);
  }

  void SolveAiYaw(float& yaw_output) {
    if (PARAM.yaw_ai_controller == YawAiController::SMC) {
      SolveAiYawSmc(yaw_output);
    } else {
      SolveAiYawLqrEso(yaw_output);
    }
  }

  void SolveAiYawLqrEso(float& yaw_output) {
    const auto YAW_LQR_ESO_OUTPUT = yaw_lqr_eso_.Calculate(
        {.theta_rad = cmd_data_.yaw,
         .omega_rad_s = cmd_data_.yaw_dot,
         .alpha_rad_s2 = cmd_data_.yaw_ddot},
        {.theta_rad = euler_.Yaw(),
         .omega_rad_s = gyro_data_.z(),
         .tau_meas_nm = motor_yaw_feedback_.torque,
         .valid = motor_yaw_->IsOnline(),
         .torque_measurement_valid = std::isfinite(motor_yaw_feedback_.torque)},
        dt_, PARAM.j_yaw, pid_yaw_omega_.OutLimit(), PARAM.yaw_k);
    if (!YAW_LQR_ESO_OUTPUT.valid ||
        !std::isfinite(YAW_LQR_ESO_OUTPUT.tau_cmd_nm)) {
      yaw_output = 0.0f;
      return;
    }
    yaw_output = YAW_LQR_ESO_OUTPUT.tau_cmd_nm;
  }

  void SolveAiYawSmc(float& yaw_output) {
    SolveSmcYaw(cmd_data_.yaw, cmd_data_.yaw_dot, cmd_data_.yaw_ddot,
                yaw_output);
  }

  void SolveManualYawSmc(const CycleFeedforward& feedforward,
                         float& yaw_output) {
    SolveSmcYaw(feedforward.yaw_angle, feedforward.yaw_omega,
                feedforward.yaw_alpha, yaw_output);
  }

  void SolveSmcYaw(float theta_ref, float omega_ref, float alpha_ref,
                   float& yaw_output) {
    const auto YAW_SMC_OUTPUT =
        yaw_smc_.Calculate({.theta_rad = theta_ref,
                            .omega_rad_s = omega_ref,
                            .alpha_rad_s2 = alpha_ref},
                           {.theta_rad = euler_.Yaw(),
                            .omega_rad_s = gyro_data_.z(),
                            .valid = motor_yaw_->IsOnline()},
                           dt_, PARAM.j_yaw);
    if (!YAW_SMC_OUTPUT.valid || !std::isfinite(YAW_SMC_OUTPUT.tau_cmd_nm)) {
      yaw_output = 0.0f;
      return;
    }
    yaw_output = YAW_SMC_OUTPUT.tau_cmd_nm;
  }

  static LibXR::Topic& YawAngleTopic() {
    static LibXR::Topic topic =
        LibXR::Topic::CreateTopic<float>("yawmotor_angle");
    return topic;
  }

  static LibXR::Topic& PitchAngleTopic() {
    static LibXR::Topic topic =
        LibXR::Topic::CreateTopic<float>("pitchmotor_angle");
    return topic;
  }

  static LibXR::Topic& YawOmegaTopic() {
    static LibXR::Topic topic =
        LibXR::Topic::CreateTopic<float>("yawmotor_omega");
    return topic;
  }

  static LibXR::Topic& PitchOmegaTopic() {
    static LibXR::Topic topic =
        LibXR::Topic::CreateTopic<float>("pitchmotor_omega");
    return topic;
  }

  static LibXR::Topic& ModeTopic() {
    static LibXR::Topic topic =
        LibXR::Topic::CreateTopic<uint8_t>("gimbal_mode");
    return topic;
  }

  static LibXR::Topic& VisionTaskTopic() {
    static LibXR::Topic topic =
        LibXR::Topic::CreateTopic<uint8_t>("vision_task");
    return topic;
  }

  static LibXR::Topic& ChassisMotionStateTopic() {
    static LibXR::Topic topic = LibXR::Topic::FindOrCreate<ChassisMotionState>(
        CHASSIS_MOTION_STATE_TOPIC_NAME, nullptr,
        CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER);
    return topic;
  }

  static void InitializeTopics() {
    YawAngleTopic();
    PitchAngleTopic();
    YawOmegaTopic();
    PitchOmegaTopic();
    ModeTopic();
    VisionTaskTopic();
  }

  void PublishVisionTask(uint8_t task) { VisionTaskTopic().Publish(task); }

  void SyncGimbalSetpoint(float yaw_rad, float pit_rad) {
    cmd_data_.yaw = yaw_rad;
    cmd_data_.pit = pit_rad;
    cmd_.SetGimbalSetpoint(yaw_rad, pit_rad);
  }

  /**
   * @brief 设置云台模式
   *
   * @param gimbal_event 云台事件类型
   */
  void SetMode(GimbalEvent gimbal_event) {
    // 辨识 DONE/ABORTED 后锁死 RELAX：忽略一切非 RELAX 模式请求，
    // 直到以 system_identify.enabled: false 重启。
    if (identify_relax_lock_ && gimbal_event != GimbalEvent::SET_MODE_RELAX) {
      return;
    }
    // 辨识进行中收到任何外部模式事件（含 lost ctrl 强制的
    // RELAX）= 操作手中止：锁存已收敛轴结果，强制回 RELAX 并上锁。
    if (current_mode_ == GimbalEvent::SET_MODE_IDENTIFY &&
        gimbal_event != GimbalEvent::SET_MODE_IDENTIFY &&
        system_identify_.IsActive()) {
      system_identify_.Abort(IdentifyAbort::EXTERNAL_EVENT);
      identify_relax_lock_ = true;
      gimbal_event = GimbalEvent::SET_MODE_RELAX;
    }
    if (gimbal_event == current_mode_) {
      return;
    }
    // 目标模式在进入分支前统一锁存：各 case 只使用形参 gimbal_event，均不读
    // current_mode_，因此提前赋值不改变任何分支行为，同时消除了逐分支重复的
    // current_mode_ = gimbal_event。switch 覆盖全部枚举值，default 仅为防御。
    current_mode_ = gimbal_event;

    switch (gimbal_event) {
      case GimbalEvent::SET_VISION_IDLE:
        PublishVisionTask(0U);
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        SyncGimbalSetpoint(euler_.Yaw(), euler_.Pitch());
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        break;
      case GimbalEvent::SET_VISION_AUTO_AIM:
        PublishVisionTask(1U);
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        SyncGimbalSetpoint(euler_.Yaw(), euler_.Pitch());
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        break;
      case GimbalEvent::SET_VISION_SMALL_BUFF:
        PublishVisionTask(2U);
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        SyncGimbalSetpoint(euler_.Yaw(), euler_.Pitch());
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        break;
      case GimbalEvent::SET_VISION_BIG_BUFF:
        PublishVisionTask(3U);
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        SyncGimbalSetpoint(euler_.Yaw(), euler_.Pitch());
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        break;
      case GimbalEvent::SET_MODE_RELAX:
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        motor_yaw_->Disable();
        motor_pit_->Disable();
        PublishVisionTask(0U);
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        SyncGimbalSetpoint(0.0f, 0.0f);
        break;
      case GimbalEvent::SET_MODE_COMMON:
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        SyncGimbalSetpoint(euler_.Yaw(), euler_.Pitch());
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        break;
      case GimbalEvent::SET_MODE_AUTOPATROL:
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        SyncGimbalSetpoint(euler_.Yaw(), euler_.Pitch());
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        patrol_ = {.pitch_center_rad = euler_.Pitch(),
                   .yaw_origin_rad = euler_.Yaw(),
                   .start_time = LibXR::Timebase::GetMilliseconds()};
        break;
      case GimbalEvent::SET_MODE_IDENTIFY:
        PublishVisionTask(0U);
        pid_pit_omega_.SetFeedForward(0.0f);
        pid_yaw_omega_.SetFeedForward(0.0f);
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        system_identify_.Begin();
        break;
      default:
        break;
    }
  }
};

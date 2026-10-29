#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
depends:
- id: GUET-PLDX/CMD
- id: GUET-PLDX/Motor
- id: GUET-PLDX/BMI088
=== END MANIFEST === */
// clang-format on

#include <cmath>

#include "CMD.hpp"
#include "Motor.hpp"
#include "SystemIdentify.hpp"
#include "YawSmc.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

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
class Gimbal {
 public:
  struct ControlParam {
    LibXR::PID<float>::Param pid_pit_angle;
    LibXR::PID<float>::Param pid_pit_omega;
    YawSmc::Config yaw_smc;
  };

  struct Param {
    uint32_t task_stack_depth;
    ControlParam manual;
    ControlParam auto_aim;
    YawSmc::OutputLimits yaw_output_limits;
    float pit_max_angle;
    float pit_min_angle;
    float pit_lc;
    float pit_theta;
    float j_pit;
    float j_yaw;
    float pit_zero;
    float yaw_zero;
    float patrol_pitch_amplitude_rad;
    float patrol_pitch_angular_rate_rad_s;
    float patrol_yaw_rate_rad_s;
    bool reverse_flag;
    LibXR::Thread::Priority thread_priority;
    SystemIdentify::Config system_identify;
    const char* euler_topic_name;
    const char* gyro_topic_name;
    const char* gimbal_cmd_topic_name;
  };

  /**
   * @brief 构造 Gimbal，创建控制线程并注册模式事件
   *
   * @param cmd 命令模块实例
   * @param motor_pit Pitch轴电机
   * @param motor_yaw Yaw轴电机
   * @param param 配置参数：控制 profile、机构、安全、巡逻与输入主题
   *
   * @note xrobot 1.x 按构造参数名检查 YAML args，配置缺项时使用声明的默认值；
   *       param 的 YAML 映射须按结构体声明顺序列出全部字段。
   */
  Gimbal(CMD& cmd, Motor& motor_pit, Motor& motor_yaw,
         const Param& param =
             {.task_stack_depth = 2048,
              .manual = {.pid_pit_angle = {.k = 0.0,
                                           .p = 0.0,
                                           .i = 0.0,
                                           .d = 0.0,
                                           .i_limit = 0.0,
                                           .out_limit = 0.0,
                                           .cycle = false},
                         .pid_pit_omega = {.k = 0.0,
                                           .p = 0.0,
                                           .i = 0.0,
                                           .d = 0.0,
                                           .i_limit = 0.0,
                                           .out_limit = 0.0,
                                           .cycle = false},
                         .yaw_smc = {}},
              .auto_aim = {.pid_pit_angle = {.k = 0.0,
                                             .p = 0.0,
                                             .i = 0.0,
                                             .d = 0.0,
                                             .i_limit = 0.0,
                                             .out_limit = 0.0,
                                             .cycle = false},
                           .pid_pit_omega = {.k = 0.0,
                                             .p = 0.0,
                                             .i = 0.0,
                                             .d = 0.0,
                                             .i_limit = 0.0,
                                             .out_limit = 0.0,
                                             .cycle = false},
                           .yaw_smc = {}},
              .yaw_output_limits = {},
              .pit_max_angle = 0.0,
              .pit_min_angle = 0.0,
              .pit_lc = 0.0,
              .pit_theta = 0.0,
              .j_pit = 0.0,
              .j_yaw = 0.0,
              .pit_zero = 0.0,
              .yaw_zero = 0.0,
              .patrol_pitch_amplitude_rad = 0.0,
              .patrol_pitch_angular_rate_rad_s = 0.0,
              .patrol_yaw_rate_rad_s = 0.0,
              .reverse_flag = false,
              .thread_priority = LibXR::Thread::Priority::MEDIUM,
              .system_identify = {.enabled = false,
                                  .axis_select = IdentifyAxis::BOTH,
                                  .arming_delay_s = 3.0,
                                  .step_torque_yaw_nm = 0.5,
                                  .step_torque_pit_nm = 0.2,
                                  .step_half_period_s = 0.5,
                                  .step_cycles = 10,
                                  .settle_time_s = 1.0,
                                  .lpf_cutoff_hz = 50.0,
                                  .alpha_gate_rad_s2 = 1.0,
                                  .rls_delta = 1000.0,
                                  .rls_lambda = 1.0,
                                  .coulomb_tanh_scale = 0.1,
                                  .omega_abort_yaw = 8.0,
                                  .omega_abort_pit = 4.0,
                                  .pit_angle_margin_rad = 0.1,
                                  .axis_timeout_s = 30.0},
              .euler_topic_name = "gimbal_euler",
              .gyro_topic_name = "gimbal_gyro",
              .gimbal_cmd_topic_name = "gimbal_cmd"})
      : cmd_(cmd),
        PARAM(param),
        manual_control_(PARAM.manual, PARAM.yaw_output_limits),
        auto_aim_control_(PARAM.auto_aim, PARAM.yaw_output_limits),
        motor_yaw_(&motor_yaw),
        motor_pit_(&motor_pit),
        system_identify_(param.system_identify) {
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          ASSERT(!in_isr);
          if (in_isr) return;
          gimbal->SetMode(static_cast<GimbalEvent>(event_id));
        },
        this);
    cmd_.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    for (uint32_t event = 0;
         event <= static_cast<uint32_t>(GimbalEvent::SET_MODE_IDENTIFY);
         ++event) {
      gimbal_event_.Register(event, callback);
    }
    thread_.Create(this, ThreadFunc, "GimbalThread", param.task_stack_depth,
                   PARAM.thread_priority);
  };

  /**
   * @brief 线程函数
   *
   * @param gimbal Gimbal实例指针
   */
  static void ThreadFunc(Gimbal* gimbal) {
    LibXR::Topic::ASyncSubscriber<CMD::GimbalCMD> cmd_suber(
        gimbal->PARAM.gimbal_cmd_topic_name);
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        gimbal->PARAM.euler_topic_name);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        gimbal->PARAM.gyro_topic_name);
    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();

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

      gimbal->Update();
      gimbal->ParseCMD();
      gimbal->Control();
      LibXR::Thread::SleepUntil(last_time, 1);
    }
  };

  /**
   * @brief 更新电机反馈及状态
   */
  void Update() {
    // 上电一次性钩子：system_identify.enabled 时自动进入辨识模式。
    // 辨识完成后锁死 RELAX（见 SetMode），须以 enabled:false 重启恢复正常使用。
    // 操作规程：右拨杆置上段（RELAX）、先开 RC 再上电——拨杆在中/下段时
    // EventBinder 位置事件会先把 current_mode_ 置为 COMMON/VISION，钩子
    // 条件不再成立，辨识不会进入；ARMING 期的 RELAX 位置事件由 SetMode
    // 放行（见 SetMode 注释）。
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
    topic_yaw_angle_.Publish(yaw_encoder_relative_angle);
    topic_pit_angle_.Publish(pitch_encoder_relative_angle);
    topic_yaw_omega_.Publish(gyro_data_.z());
    topic_pit_omega_.Publish(gyro_data_.y());
    uint8_t mode = static_cast<uint8_t>(current_mode_);
    topic_mode_.Publish(mode);
  }

  /**
   * @brief 解析云台控制命令
   */
  void ParseCMD() {
    const auto CTRL_MODE = cmd_.GetCtrlMode();
    // 自动控制、视觉在线且处于视觉任务模式时切换到自瞄 profile。
    const bool AI_ACTIVE =
        CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && cmd_.GetAIGimbalStatus() &&
        (current_mode_ == GimbalEvent::SET_VISION_AUTO_AIM ||
         current_mode_ == GimbalEvent::SET_VISION_SMALL_BUFF ||
         current_mode_ == GimbalEvent::SET_VISION_BIG_BUFF);
    if (AI_ACTIVE != ai_gimbal_reference_active_) {
      ai_gimbal_reference_active_ = AI_ACTIVE;
      auto& control = ActiveControl();
      control.Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                    gyro_data_.z(), last_applied_yaw_torque_nm_);
      target_yaw_cmd_ = euler_.Yaw();
      target_pit_cmd_ = euler_.Pitch();
      target_pit_dot_ = 0.0f;
      target_pit_ddot_ = 0.0f;
      target_yaw_dot_ = 0.0f;
      target_yaw_ddot_ = 0.0f;
      control_profile_just_changed_ = true;
    }
    if (control_profile_just_changed_) {
      control_profile_just_changed_ = false;
      return;
    }

    // 操作手控制：摇杆直接积分目标角。
    if (CTRL_MODE == CMD::Mode::CMD_OP_CTRL) {
      target_yaw_cmd_ += cmd_data_.yaw * dt_ * GIMBAL_MAX_SPEED * 1.0f;
      target_pit_cmd_ += cmd_data_.pit * dt_ * GIMBAL_MAX_SPEED * 1.0f;
      target_pit_dot_ = 0.0f;
      target_pit_ddot_ = 0.0f;
      target_yaw_dot_ = 0.0f;
      target_yaw_ddot_ = 0.0f;
      return;
    }

    if (ai_gimbal_reference_active_) {
      target_pit_cmd_ = cmd_data_.pit;
      target_pit_dot_ = cmd_data_.pit_dot;
      target_pit_ddot_ = cmd_data_.pit_ddot;
      target_yaw_cmd_ = cmd_data_.yaw;
      target_yaw_dot_ = cmd_data_.yaw_dot;
      target_yaw_ddot_ = cmd_data_.yaw_ddot;
      return;
    }

    // 自动巡逻只在操作手未接管时生效。
    if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
      const float ELAPSED_S =
          static_cast<float>(
              (LibXR::Timebase::GetMilliseconds() - patrol_.start_time)
                  .ToMillisecond()) /
          1000.0f;
      constexpr float TWO_OVER_PI = 2.0f / static_cast<float>(LibXR::PI);
      target_pit_cmd_ =
          patrol_.pitch_center_rad +
          PARAM.patrol_pitch_amplitude_rad * TWO_OVER_PI *
              std::asin(
                  std::sin(PARAM.patrol_pitch_angular_rate_rad_s * ELAPSED_S));
      target_yaw_cmd_ =
          patrol_.yaw_origin_rad + PARAM.patrol_yaw_rate_rad_s * ELAPSED_S;
      target_pit_dot_ = 0.0f;
      target_pit_ddot_ = 0.0f;
      target_yaw_dot_ = PARAM.patrol_yaw_rate_rad_s;
      target_yaw_ddot_ = 0.0f;
      return;
    }

    // 自动控制下的手动回退：Yaw 方向与操作手相反。
    target_yaw_cmd_ -= cmd_data_.yaw * dt_ * GIMBAL_MAX_SPEED * 1.0f;
    target_pit_cmd_ += cmd_data_.pit * dt_ * GIMBAL_MAX_SPEED * 1.0f;
    // 手动速度/加速度前馈置零；Solve 对目标角速度逐周期差分计算惯性项。
    target_pit_dot_ = 0.0f;
    target_pit_ddot_ = 0.0f;
    target_yaw_dot_ = 0.0f;
    target_yaw_ddot_ = 0.0f;
  }

  /**
   * @brief 云台控制计算与输出
   */
  void Control() {
    float pit_output = 0.0f;
    float yaw_output = 0.0f;

    if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
      motor_yaw_->Relax();
      CommitAppliedYawTorque(0.0f);
      motor_pit_->Relax();
      return;
    }

    if (current_mode_ == GimbalEvent::SET_MODE_IDENTIFY) {
      ControlIdentify();
      return;
    }

    PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle,
               PARAM.pit_max_angle, PARAM.pit_min_angle,
               PARAM.reverse_flag ? 1.0f : -1.0f);
    Solve(pit_output, yaw_output);

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
        if (motor == motor_yaw_) {
          CommitAppliedYawTorque(cmd.torque);
        }
      }
    };

    motor_control(motor_pit_, motor_pit_feedback_, pit_motor_cmd);
    motor_control(motor_yaw_, motor_yaw_feedback_, yaw_motor_cmd);
  }

  LibXR::Event& GetEvent() { return gimbal_event_; }

 private:
  struct ControlState {
    LibXR::PID<float> pid_pit_angle;
    LibXR::PID<float> pid_pit_omega;
    YawSmc yaw_smc;
    float last_pit_omega = 0.0f;

    ControlState(const ControlParam& param,
                 const YawSmc::OutputLimits& output_limits)
        : pid_pit_angle(param.pid_pit_angle),
          pid_pit_omega(param.pid_pit_omega),
          yaw_smc(param.yaw_smc, output_limits) {}

    void Reset(float pitch, float pitch_omega, float yaw, float yaw_omega,
               float applied_yaw_torque) {
      pid_pit_omega.SetFeedForward(0.0f);
      pid_pit_angle.Reset();
      pid_pit_omega.Reset();
      pid_pit_angle.Calculate(pitch, pitch, 1.0f);
      pid_pit_omega.Calculate(pitch_omega, pitch_omega, 1.0f);
      last_pit_omega = 0.0f;
      yaw_smc.Reset(yaw, yaw_omega, applied_yaw_torque);
    }
  };

  CMD& cmd_;
  const Param PARAM;
  ControlState manual_control_;
  ControlState auto_aim_control_;
  Motor* motor_yaw_;
  Motor* motor_pit_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pit_feedback_;

  CMD::GimbalCMD cmd_data_;

  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  /// 自动巡逻会话状态：进入 SET_MODE_AUTOPATROL
  /// 时一次性锁存起点，巡逻期间只读。
  struct PatrolState {
    float pitch_center_rad = 0.0f;
    float yaw_origin_rad = 0.0f;
    LibXR::MillisecondTimestamp start_time = 0.0f;
  };

  PatrolState patrol_{};
  static constexpr float GIMBAL_MAX_SPEED =
      static_cast<float>(LibXR::TWO_PI) * 2.0f;
  float target_pit_cmd_ = 0.0f;
  float target_pit_dot_ = 0.0f;
  float target_pit_ddot_ = 0.0f;
  LibXR::CycleValue<float> target_yaw_cmd_ = 0.0f;
  float target_yaw_dot_ = 0.0f;
  float target_yaw_ddot_ = 0.0f;
  bool ai_gimbal_reference_active_ = false;
  bool control_profile_just_changed_ = false;
  float last_applied_yaw_torque_nm_ = 0.0f;
  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  SystemIdentify system_identify_;
  bool identify_boot_checked_ = false;
  bool identify_relax_lock_ = false;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_pit_angle_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_angle");
  LibXR::Topic topic_yaw_omega_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_omega");
  LibXR::Topic topic_pit_omega_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_omega");
  LibXR::Topic topic_mode_ = LibXR::Topic::CreateTopic<uint8_t>("gimbal_mode");
  LibXR::Topic topic_vision_task_ =
      LibXR::Topic::CreateTopic<uint8_t>("vision_task");
  LibXR::Thread thread_;

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

  /**
   * @brief 系统辨识模式控制输出（开环力矩双向阶跃 + RLS 在线辨识 J/B）。
   *
   * 生效轴输出激励力矩，非生效轴 Relax；pitch 角度限值取 Param
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

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 and fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
        if (motor == motor_yaw_) {
          CommitAppliedYawTorque(cmd.torque);
        }
      }
    };
    const auto IDENTIFY_CMD = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = TAU});

    if (ACTIVE == SystemIdentify::ActiveAxis::YAW) {
      motor_pit_->Relax();
      motor_control(motor_yaw_, motor_yaw_feedback_, IDENTIFY_CMD);
    } else if (ACTIVE == SystemIdentify::ActiveAxis::PITCH) {
      motor_yaw_->Relax();
      CommitAppliedYawTorque(0.0f);
      motor_control(motor_pit_, motor_pit_feedback_, IDENTIFY_CMD);
    } else {
      motor_yaw_->Relax();
      CommitAppliedYawTorque(0.0f);
      motor_pit_->Relax();
    }

    if (system_identify_.State() == IdentifyState::DONE ||
        system_identify_.State() == IdentifyState::ABORTED) {
      identify_relax_lock_ = true;
      SetMode(GimbalEvent::SET_MODE_RELAX);
    }
  }

  ControlState& ActiveControl() {
    return ai_gimbal_reference_active_ ? auto_aim_control_ : manual_control_;
  }

  void CommitAppliedYawTorque(float torque) {
    if (!std::isfinite(torque)) {
      return;
    }
    last_applied_yaw_torque_nm_ = torque;
    ActiveControl().yaw_smc.CommitAppliedTorque(torque);
  }

  /**
   * @brief 解算PID控制输出
   */
  void Solve(float& pit_output, float& yaw_output) {
    auto& control = ActiveControl();
    const float PIT_ERROR = target_pit_cmd_ - euler_.Pitch();
    // sp=误差、fb=0 与 sp=目标、fb=角度 等价，故 Pitch D 项使用实测角速度。
    const float PIT_ANGLE_LOOP_OMEGA =
        control.pid_pit_angle.Calculate(PIT_ERROR, 0.0f, gyro_data_.y(), dt_);
    const float TARGET_PIT_OMEGA = PIT_ANGLE_LOOP_OMEGA + target_pit_dot_;
    const float PITCH_FEEDFORWARD =
        JFeedforward(TARGET_PIT_OMEGA, control.last_pit_omega, dt_,
                     PARAM.j_pit) +
        PARAM.j_pit * target_pit_ddot_ -
        PARAM.pit_lc * sinf(euler_.Pitch() + PARAM.pit_theta);
    control.pid_pit_omega.SetFeedForward(PITCH_FEEDFORWARD);
    pit_output =
        control.pid_pit_omega.Calculate(TARGET_PIT_OMEGA, gyro_data_.y(), dt_);
    control.last_pit_omega = TARGET_PIT_OMEGA;

    SolveSmcYaw(control.yaw_smc, target_yaw_cmd_, target_yaw_dot_,
                target_yaw_ddot_, yaw_output);
  }

  /**
   * @brief 计算转动惯量前馈 J * (target_omega - last_omega) / dt。
   *        Compute the moment-of-inertia feedforward.
   *
   * @param target_omega 目标角速度 (rad/s)
   * @param last_omega 上一周期的目标角速度 (rad/s)
   * @param dt 控制周期 (s)
   * @param j 转动惯量 (kg*m^2)
   * @return 前馈力矩 (N*m)
   */
  static float JFeedforward(float target_omega, float last_omega, float dt,
                            float j) {
    return j * (target_omega - last_omega) / dt;
  }

  void SolveSmcYaw(YawSmc& yaw_smc, float theta_ref, float omega_ref,
                   float alpha_ref, float& yaw_output) {
    const auto YAW_SMC_OUTPUT =
        yaw_smc.Calculate({.theta_rad = theta_ref,
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
    // ARMING 期放行 RELAX：EventBinder 的拨杆电平映射在 RC 上线/复联
    // 首帧会补发"右拨杆上段=RELAX"的位置事件（DR16 首帧不走失联边沿
    // 基线），这是安全默认位置而非操作手接管意图，忽略之，倒计时不
    // 中断；RC 失联由 StepArming 的 rc_online 门控兜底（离线持续重置
    // 倒计时）。COMMON/VISION 仍是明确接管意图，照常中止。
    // 操作规程：辨识时右拨杆置上段（RELAX）、先开 RC 再上云台板电。
    if (current_mode_ == GimbalEvent::SET_MODE_IDENTIFY &&
        gimbal_event == GimbalEvent::SET_MODE_RELAX &&
        system_identify_.State() == IdentifyState::ARMING) {
      return;
    }
    // 辨识进行中收到任何外部模式事件（激励/静置期含 lost ctrl 强制的
    // RELAX）= 操作手中止：锁存已收敛轴结果，强制回 RELAX 并上锁。
    // （ARMING 期的 RELAX 已在上方放行，不会走到这里。）
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

    // LibXR::Topic::Publish 只接受左值，任务号须经具名变量发布。
    uint8_t vision_task = 0U;
    switch (gimbal_event) {
      case GimbalEvent::SET_MODE_RELAX:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        motor_yaw_->Disable();
        CommitAppliedYawTorque(0.0f);
        motor_pit_->Disable();
        topic_vision_task_.Publish(vision_task);
        target_yaw_cmd_ = 0.0f;
        target_pit_cmd_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_COMMON:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_AUTOPATROL:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        patrol_ = {.pitch_center_rad = euler_.Pitch(),
                   .yaw_origin_rad = euler_.Yaw(),
                   .start_time = LibXR::Timebase::GetMilliseconds()};
        break;
      case GimbalEvent::SET_VISION_IDLE:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        topic_vision_task_.Publish(vision_task);
        break;
      case GimbalEvent::SET_VISION_AUTO_AIM:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        vision_task = 1U;
        topic_vision_task_.Publish(vision_task);
        break;
      case GimbalEvent::SET_VISION_SMALL_BUFF:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        vision_task = 2U;
        topic_vision_task_.Publish(vision_task);
        break;
      case GimbalEvent::SET_VISION_BIG_BUFF:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        vision_task = 3U;
        topic_vision_task_.Publish(vision_task);
        break;
      case GimbalEvent::SET_MODE_IDENTIFY:
        ActiveControl().Reset(euler_.Pitch(), gyro_data_.y(), euler_.Yaw(),
                              gyro_data_.z(), last_applied_yaw_torque_nm_);
        target_yaw_cmd_ = euler_.Yaw();
        target_pit_cmd_ = euler_.Pitch();
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        topic_vision_task_.Publish(vision_task);
        system_identify_.Begin();
        break;
      default:
        break;
    }
  }
};

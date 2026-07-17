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
      out_limit: 0.0
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
  - pit_max_angle: 0.0
  - pit_min_angle: 0.0
  - pit_lc: 0.0
  - pit_theta: 0.0
  - yaw_k: 0.0
  - j_pit: 0.0
  - j_yaw: 0.0
  - pit_zero: 0.0
  - yaw_zero: 0.0
  - patrol_range: 0.0
  - patrol_omega: 0.0
  - pit_reverse_flag: false
  - referee: '@&ref'
  - thread_priority: LibXR::Thread::Priority::MEDIUM
  - rotor_ff_enabled: false
  - yaw_lqr_eso:
      j_kg_m2: 0.03
      b_nms_rad: 0.0
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
      torque_min_nm: -2.223
      torque_max_nm: 2.223
      torque_slew_rate_nm_s: 1000.0
      eso_enable: true
      eso_comp_enable: false
      coulomb_enable: false
      lqi_enable: false
      torque_bias_enable: false
      torque_slew_enable: true
template_args: []
required_hardware: []
depends:
  - pldx/CMD
  - pldx/Motor
  - pldx/BMI088
  - pldx/Referee
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "Referee.hpp"
#include "YawLqrEso.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

#define UI_GIMBAL_LAYER 3

static constexpr float GIMBAL_MAX_SPEED =
    static_cast<float>(LibXR::TWO_PI) * 2.0f;
static constexpr uint32_t CHASSIS_MODE_ROTOR = 2U;
enum class GimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
  SET_MODE_AUTOPATROL,
  SET_MODE_LOW_SENSITIVITY
};
class Gimbal : public LibXR::Application {
 public:
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
   * @param reverse_flag Pitch轴反转标志
   * @param rotor_ff_enabled
   *
   * 是否启用小陀螺模式Yaw轴角速度前馈
   * @param yaw_lqr_eso AI Yaw LQR/ESO参数
   */
  Gimbal(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
      uint32_t task_stack_depth, LibXR::PID<float>::Param pid_yaw_angle,
      LibXR::PID<float>::Param pid_yaw_omega,
      LibXR::PID<float>::Param pid_pit_angle,
      LibXR::PID<float>::Param pid_pit_omega, Motor* motor_pit,
      Motor* motor_yaw, float pit_max_angle, float pit_min_angle, float pit_lc,
      float pit_theta, float yaw_k, float j_pit, float j_yaw, float pit_zero,
      float yaw_zero, float patrol_range, float patrol_omega, bool reverse_flag,
      Referee* referee,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,
      bool rotor_ff_enabled = false, YawLqrEso::Config yaw_lqr_eso = {})
      : cmd_(cmd),
        pid_yaw_angle_(pid_yaw_angle),
        pid_yaw_omega_(pid_yaw_omega),
        pid_pit_angle_(pid_pit_angle),
        pid_pit_omega_(pid_pit_omega),
        motor_yaw_(motor_yaw),
        motor_pit_(motor_pit),
        pit_max_angle_(pit_max_angle),
        pit_min_angle_(pit_min_angle),
        pit_lc_(pit_lc),
        pit_theta_(pit_theta),
        yaw_k_(yaw_k),
        j_pit_(j_pit),
        j_yaw_(j_yaw),
        pit_zero_(pit_zero),
        yaw_zero_(yaw_zero),
        patrol_range_(patrol_range),
        patrol_omega_(patrol_omega),
        reverse_flag_(reverse_flag ? 1.0f : -1.0f),
        referee_(referee),
        rotor_ff_enabled_(rotor_ff_enabled),
        yaw_lqr_eso_config_(yaw_lqr_eso),
        chassis_gyro_z_topic_(LibXR::Topic::FindOrCreate<float>(
            "chassis_gyro_z", nullptr, false)),
        dualboard_chassis_mode_topic_(LibXR::Topic::FindOrCreate<uint32_t>(
            "dualboard_chassis_mode", nullptr, true)) {
    UNUSED(app);
    UNUSED(referee_);

    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   thread_priority);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
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
    cmd_.GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_RELAX),
                           callback);
    gimbal_event_.Register(static_cast<uint32_t>(GimbalEvent::SET_MODE_COMMON),
                           callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_AUTOPATROL), callback);
    gimbal_event_.Register(
        static_cast<uint32_t>(GimbalEvent::SET_MODE_LOW_SENSITIVITY), callback);
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
    LibXR::Topic::ASyncSubscriber<float> chassis_gyro_z_suber(
        LibXR::Topic(gimbal->chassis_gyro_z_topic_));
    LibXR::Topic::ASyncSubscriber<uint32_t> dualboard_chassis_mode_suber(
        LibXR::Topic(gimbal->dualboard_chassis_mode_topic_));
    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();
    chassis_gyro_z_suber.StartWaiting();
    dualboard_chassis_mode_suber.StartWaiting();

    gimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true) {
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (euler_suber.Available()) {
        gimbal->euler_ = euler_suber.GetData();
        gimbal->euler_.Pitch() *= -1.0f;
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        gimbal->gyro_data_ = gyro_suber.GetData();
        gimbal->gyro_data_.y() *= -1.0f;
        gyro_suber.StartWaiting();
      }
      if (chassis_gyro_z_suber.Available()) {
        gimbal->chassis_gyro_z_ = chassis_gyro_z_suber.GetData();
        chassis_gyro_z_suber.StartWaiting();
      }
      if (dualboard_chassis_mode_suber.Available()) {
        gimbal->dualboard_chassis_mode_ =
            dualboard_chassis_mode_suber.GetData();
        dualboard_chassis_mode_suber.StartWaiting();
      }

      gimbal->Update();
      gimbal->ParseCMD();
      gimbal->Control();
      LibXR::Thread::Sleep(2);
    }
  };

  /**
   * @brief 更新电机反馈及状态
   */
  void Update() {
    auto yaw_update_status = motor_yaw_->Update();
    auto pit_update_status = motor_pit_->Update();
    motor_feedback_online_ = yaw_update_status == LibXR::ErrorCode::OK &&
                             pit_update_status == LibXR::ErrorCode::OK;
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_pit_feedback_ = motor_pit_->GetFeedback();

    const auto NOW = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (NOW - this->last_online_time_).ToSecondf();
    this->last_online_time_ = NOW;
    abs_angle_pit_ = motor_pit_feedback_.abs_angle - pit_zero_;
    abs_angle_yaw_ = motor_yaw_feedback_.abs_angle - yaw_zero_;

    topic_yaw_angle_.Publish(abs_angle_yaw_);
    topic_pit_angle_.Publish(abs_angle_pit_);
  }

  /**
   * @brief 解析云台控制命令
   */
  void ParseCMD() {
    const auto CTRL_MODE = cmd_.GetCtrlMode();
    const bool AI_GIMBAL_ACTIVE = cmd_.GetAIGimbalStatus();
    const bool AI_YAW_ACTIVE =
        CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && AI_GIMBAL_ACTIVE;
    ai_yaw_active_ = AI_YAW_ACTIVE;

    if (CTRL_MODE == CMD::Mode::CMD_OP_CTRL) {
      if (current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY) {
        const float PIT_OPERATOR_RATE = cmd_data_.pit * GIMBAL_MAX_SPEED * 0.1f;
        target_pit_cmd_ += PIT_OPERATOR_RATE * dt_;
        target_pit_dot_ = PIT_OPERATOR_RATE;
        target_pit_ddot_ = 0.0f;
      } else {
        const float PIT_OPERATOR_RATE = cmd_data_.pit * GIMBAL_MAX_SPEED;
        target_pit_cmd_ += PIT_OPERATOR_RATE * dt_;
        target_pit_dot_ = PIT_OPERATOR_RATE;
        target_pit_ddot_ = 0.0f;
      }
    } else {
      if (AI_GIMBAL_ACTIVE) {
        target_pit_cmd_ = cmd_data_.pit;
        target_pit_dot_ = cmd_data_.pit_dot;
        target_pit_ddot_ = cmd_data_.pit_ddot;
      } else {
        if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
          target_pit_cmd_ -=
              patrol_range_ * (2 / M_PI) *
              asin(sin(patrol_omega_ * (LibXR::Timebase::GetMilliseconds() -
                                        patrol_start_time))) /
              1000.0f;
          target_pit_dot_ = 0.0f;
          target_pit_ddot_ = 0.0f;
        } else {
          const float PIT_OPERATOR_RATE = cmd_data_.pit * GIMBAL_MAX_SPEED;
          target_pit_cmd_ += PIT_OPERATOR_RATE * dt_;
          target_pit_dot_ = PIT_OPERATOR_RATE;
          target_pit_ddot_ = 0.0f;
        }
      }
    }

    if (!ai_yaw_active_) {
      if (CTRL_MODE == CMD::Mode::CMD_OP_CTRL) {
        const float YAW_SENSITIVITY =
            current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY ? 0.1f
                                                                   : 1.0f;
        const float YAW_OPERATOR_RATE =
            cmd_data_.yaw * GIMBAL_MAX_SPEED * YAW_SENSITIVITY;
        target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
        target_yaw_dot_ = YAW_OPERATOR_RATE;
        target_yaw_ddot_ = 0.0f;
      } else if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
        target_yaw_cmd_ += 1.0f * dt_;
        target_yaw_dot_ = 1.0f;
        target_yaw_ddot_ = 0.0f;
      } else {
        const float YAW_OPERATOR_RATE = -cmd_data_.yaw * GIMBAL_MAX_SPEED;
        target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
        target_yaw_dot_ = YAW_OPERATOR_RATE;
        target_yaw_ddot_ = 0.0f;
      }
    }
  }

  /**
   * @brief 云台控制计算与输出
   */
  void Control() {
    if (!motor_feedback_online_) {
      // 反馈离线时立即切松弛，避免继续使用旧反馈闭环输出。
      SetMode(GimbalEvent::SET_MODE_RELAX);
    }

    pit_output_ = 0.0f;
    yaw_output_ = 0.0f;

    if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
      pid_pit_omega_.SetFeedForward(0.0f);
      pid_yaw_omega_.SetFeedForward(0.0f);
      last_pit_angle_loop_omega_ = 0.0f;
      last_yaw_angle_loop_omega_ = 0.0f;
      motor_yaw_->Relax();
      motor_pit_->Relax();
      return;
    }

    /*仅用于调试极性()*/
    this->torque_ = -this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);

    PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle,
               pit_max_angle_, pit_min_angle_, reverse_flag_);
    Solve();

    auto yaw_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = yaw_output_});
    auto pit_motor_cmd = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE, .torque = pit_output_});

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
  float torque_;

  Motor::Feedback motor_yaw_feedback_;
  Motor::Feedback motor_pit_feedback_;
  bool motor_feedback_online_ = true;

  CMD::GimbalCMD cmd_data_;
  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_pit_angle_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_angle");

  float pit_max_angle_ = 0.0f;
  float pit_min_angle_ = 0.0f;
  float pit_lc_ = 0.0f;
  float pit_theta_ = 0.0f;
  float yaw_k_ = 0.0f;
  float target_yaw_dot_ = 0.0f;
  float target_yaw_ddot_ = 0.0f;
  float target_pit_dot_ = 0.0f;
  float target_pit_ddot_ = 0.0f;
  float j_pit_ = 0.0f;
  float j_yaw_ = 0.0f;
  LibXR::CycleValue<float> pit_zero_ = 0.0f;
  LibXR::CycleValue<float> yaw_zero_ = 0.0f;
  float patrol_range_ = 0.0f;
  float patrol_omega_ = 0.0f;
  float target_pit_cmd_ = 0.0f;
  LibXR::CycleValue<float> target_yaw_cmd_ = 0.0f;
  float abs_angle_yaw_ = 0.0f;
  float abs_angle_pit_ = 0.0f;
  float last_pit_angle_loop_omega_ = 0.0f;
  float last_yaw_angle_loop_omega_ = 0.0f;
  float pit_output_ = 0.0f;
  float yaw_output_ = 0.0f;
  float reverse_flag_ = 1.0f;
  LibXR::MillisecondTimestamp patrol_start_time = 0.0f;
  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  Referee* referee_;
  bool rotor_ff_enabled_ = false;
  YawLqrEso::Config yaw_lqr_eso_config_{};
  YawLqrEso yaw_lqr_eso_{};
  bool ai_yaw_active_ = false;
  bool yaw_lqr_eso_reset_pending_ = true;
  float last_submitted_yaw_torque_nm_ = 0.0f;
  bool last_submitted_yaw_torque_valid_ = false;
  float chassis_gyro_z_ = 0.0f;
  uint32_t dualboard_chassis_mode_ = 0U;
  LibXR::Topic::TopicHandle chassis_gyro_z_topic_;
  LibXR::Topic::TopicHandle dualboard_chassis_mode_topic_;
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

  void ClearSubmittedYawTorqueLedger() {
    last_submitted_yaw_torque_nm_ = 0.0f;
    last_submitted_yaw_torque_valid_ = false;
  }

  void ControlYawMotor(const Motor::MotorCmd& command) {
    if (motor_yaw_feedback_.state == 0) {
      motor_yaw_->Enable();
      ClearSubmittedYawTorqueLedger();
    } else if (motor_yaw_feedback_.state != 1) {
      motor_yaw_->ClearError();
      ClearSubmittedYawTorqueLedger();
    } else {
      motor_yaw_->Control(command);
      last_submitted_yaw_torque_nm_ = command.torque;
      last_submitted_yaw_torque_valid_ = true;
      yaw_lqr_eso_.CommitAppliedTorque(command.torque);
    }
  }

  /**
   * @brief 解算PID控制输出
   */
  void Solve() {
    const float PIT_ERROR = target_pit_cmd_ - euler_.Pitch();
    const float PIT_ANGLE_LOOP_OMEGA =
        pid_pit_angle_.Calculate(PIT_ERROR, 0.0f, dt_);
    const float TARGET_PIT_OMEGA = PIT_ANGLE_LOOP_OMEGA + target_pit_dot_;
    const float PIT_ALPHA =
        (PIT_ANGLE_LOOP_OMEGA - last_pit_angle_loop_omega_) / dt_ +
        target_pit_ddot_;
    const float PITCH_FEEDFORWARD =
        j_pit_ * PIT_ALPHA -
        this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);
    pid_pit_omega_.SetFeedForward(PITCH_FEEDFORWARD);
    pit_output_ =
        pid_pit_omega_.Calculate(TARGET_PIT_OMEGA, gyro_data_.y(), dt_);
    last_pit_angle_loop_omega_ = PIT_ANGLE_LOOP_OMEGA;

    if (ai_yaw_active_) {
      SolveAiYaw();
    } else {
      SolveLegacyYaw();
    }
  }

  void SolveLegacyYaw() {
    const float YAW_ERROR = target_yaw_cmd_ - euler_.Yaw();
    const float YAW_ANGLE_LOOP_OMEGA =
        pid_yaw_angle_.Calculate(YAW_ERROR, 0.0f, dt_);
    const float TARGET_YAW_OMEGA = YAW_ANGLE_LOOP_OMEGA + target_yaw_dot_;
    const float YAW_ALPHA =
        (YAW_ANGLE_LOOP_OMEGA - last_yaw_angle_loop_omega_) / dt_ +
        target_yaw_ddot_;
    const bool ROTOR_FF_ACTIVE =
        rotor_ff_enabled_ && dualboard_chassis_mode_ == CHASSIS_MODE_ROTOR;
    const float YAW_MOTOR_OMEGA_REF =
        ROTOR_FF_ACTIVE ? TARGET_YAW_OMEGA - chassis_gyro_z_ : TARGET_YAW_OMEGA;
    const float YAW_FEEDFORWARD =
        j_yaw_ * YAW_ALPHA + yaw_k_ * YAW_MOTOR_OMEGA_REF;
    pid_yaw_omega_.SetFeedForward(YAW_FEEDFORWARD);
    yaw_output_ =
        pid_yaw_omega_.Calculate(TARGET_YAW_OMEGA, gyro_data_.z(), dt_);
    last_yaw_angle_loop_omega_ = YAW_ANGLE_LOOP_OMEGA;
  }

  void SolveAiYaw() {
    if (yaw_lqr_eso_reset_pending_) {
      const float PREVIOUS_TORQUE = last_submitted_yaw_torque_valid_
                                        ? last_submitted_yaw_torque_nm_
                                        : 0.0f;
      yaw_lqr_eso_.Reset(euler_.Yaw(), gyro_data_.z(), PREVIOUS_TORQUE);
    }
    const auto YAW_LQR_ESO_OUTPUT = yaw_lqr_eso_.Calculate(
        yaw_lqr_eso_config_,
        {.theta_rad = cmd_data_.yaw,
         .omega_rad_s = cmd_data_.yaw_dot,
         .alpha_rad_s2 = cmd_data_.yaw_ddot},
        {.theta_rad = euler_.Yaw(),
         .omega_rad_s = gyro_data_.z(),
         .tau_meas_nm = motor_yaw_feedback_.torque,
         .valid = motor_feedback_online_,
         .torque_measurement_valid = std::isfinite(motor_yaw_feedback_.torque)},
        dt_);
    if (!YAW_LQR_ESO_OUTPUT.valid ||
        !std::isfinite(YAW_LQR_ESO_OUTPUT.tau_cmd_nm)) {
      yaw_output_ = 0.0f;
      yaw_lqr_eso_reset_pending_ = true;
      return;
    }
    yaw_lqr_eso_reset_pending_ = false;
    yaw_output_ = YAW_LQR_ESO_OUTPUT.tau_cmd_nm;
  }

  /**
   * @brief 设置云台模式
   *
   * @param gimbal_event 云台事件类型
   */
  void SetMode(GimbalEvent gimbal_event) {
    if (gimbal_event == current_mode_) {
      return;
    };
    // 如果是在 SET_MODE_COMMON 和 SET_MODE_LOW_SENSITIVITY
    // 之间切换，不重置任何变量
    if ((current_mode_ == GimbalEvent::SET_MODE_COMMON &&
         gimbal_event == GimbalEvent::SET_MODE_LOW_SENSITIVITY) ||
        (current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY &&
         gimbal_event == GimbalEvent::SET_MODE_COMMON)) {
      current_mode_ = gimbal_event;
      return;
    }
    current_mode_ = gimbal_event;

    pid_pit_omega_.SetFeedForward(0.0f);
    pid_yaw_omega_.SetFeedForward(0.0f);
    pit_output_ = 0.0f;
    yaw_output_ = 0.0f;
    last_pit_angle_loop_omega_ = 0.0f;
    last_yaw_angle_loop_omega_ = 0.0f;

    switch (gimbal_event) {
      case GimbalEvent::SET_MODE_RELAX:
        motor_yaw_->Disable();
        motor_pit_->Disable();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        target_pit_cmd_ = 0.0f;
        target_yaw_cmd_ = 0.0f;
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_COMMON:
        target_pit_cmd_ = euler_.Pitch();
        target_yaw_cmd_ = euler_.Yaw();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_AUTOPATROL:
        patrol_start_time = LibXR::Timebase::GetMilliseconds();
        target_pit_cmd_ = euler_.Pitch();
        target_yaw_cmd_ = euler_.Yaw();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        break;
      case GimbalEvent::SET_MODE_LOW_SENSITIVITY:
        target_pit_cmd_ = euler_.Pitch();
        target_yaw_cmd_ = euler_.Yaw();
        pid_pit_angle_.Reset();
        pid_pit_omega_.Reset();
        pid_yaw_angle_.Reset();
        pid_yaw_omega_.Reset();
        target_yaw_dot_ = 0.0f;
        target_yaw_ddot_ = 0.0f;
        target_pit_dot_ = 0.0f;
        target_pit_ddot_ = 0.0f;
        break;
      default:
        break;
    }
  }
};

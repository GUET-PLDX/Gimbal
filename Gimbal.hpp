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
  - pit_max_angle: 0.0
  - pit_min_angle: 0.0
  - pit_lc: 0.0
  - pit_theta: 0.0
  - yaw_k: 0.0
  - j_pit: 0.0
  - j_yaw: 0.0
  - pit_zero: 0.0
  - yaw_zero: 0.0
  - patrol_pitch_amplitude_rad: 0.0
  - patrol_pitch_angular_rate_rad_s: 0.0
  - patrol_yaw_rate_rad_s: 0.0
  - pit_reverse_flag: false
  - thread_priority: LibXR::Thread::Priority::MEDIUM
  - rotor_ff_enabled: false
  - yaw_manual_controller: YawManualController::PID
  - yaw_ai_controller: YawAiController::LQR_ESO
  - yaw_lqr_eso:
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
      torque_slew_rate_nm_s: 1000.0
      eso_enable: true
      eso_comp_enable: false
      coulomb_enable: false
      lqi_enable: false
      torque_bias_enable: false
      torque_slew_enable: true
  - yaw_smc:
      j_kg_m2: 0.03
      c: 20.0
      k: 120.0
      epsilon: 0.5
      q: 21.0
      p: 27.0
      error_deadband_rad: 0.0
      ftsmc_switch_rad: 0.0174533
      sat_boundary: 1.0
      torque_soft_limit_nm: 2.0
      torque_min_nm: -2.223
      torque_max_nm: 2.223
      torque_slew_rate_nm_s: 1000.0
      ftsmc_enable: true
      torque_slew_enable: true
template_args: []
required_hardware: []
depends:
  - pldx/CMD
  - pldx/Motor
  - pldx/BMI088
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cmath>

#include "CMD.hpp"
#include "ChassisMotionState.hpp"
#include "GimbalInputGuard.hpp"
#include "Motor.hpp"
#include "PatrolTrajectory.hpp"
#include "YawLqrEso.hpp"
#include "YawSmc.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "queue.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

static constexpr float GIMBAL_MAX_SPEED =
    static_cast<float>(LibXR::TWO_PI) * 2.0f;
static constexpr uint32_t IMU_TIMEOUT_US = 50000U;
enum class GimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
  SET_MODE_AUTOPATROL,
  SET_MODE_LOW_SENSITIVITY,
  SET_VISION_IDLE,
  SET_VISION_AUTO_AIM,
  SET_VISION_SMALL_BUFF,
  SET_VISION_BIG_BUFF
};
struct GimbalModeRequest {
  GimbalEvent mode;
  uint32_t sequence;
  uint32_t fresh_epoch;
};
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_RELAX) == 0U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_COMMON) == 1U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_AUTOPATROL) == 2U);
static_assert(static_cast<uint8_t>(GimbalEvent::SET_MODE_LOW_SENSITIVITY) ==
              3U);
enum class YawManualController : uint8_t { PID, SMC };
enum class YawAiController : uint8_t { LQR_ESO, SMC };
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
   */
  Gimbal(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
      uint32_t task_stack_depth, LibXR::PID<float>::Param pid_yaw_angle,
      LibXR::PID<float>::Param pid_yaw_omega,
      LibXR::PID<float>::Param pid_pit_angle,
      LibXR::PID<float>::Param pid_pit_omega, Motor* motor_pit,
      Motor* motor_yaw, float pit_max_angle, float pit_min_angle, float pit_lc,
      float pit_theta, float yaw_k, float j_pit, float j_yaw, float pit_zero,
      float yaw_zero, float patrol_pitch_amplitude_rad,
      float patrol_pitch_angular_rate_rad_s, float patrol_yaw_rate_rad_s,
      bool reverse_flag,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,
      bool rotor_ff_enabled = false,
      YawManualController yaw_manual_controller = YawManualController::PID,
      YawAiController yaw_ai_controller = YawAiController::LQR_ESO,
      YawLqrEso::Config yaw_lqr_eso = {}, YawSmc::Config yaw_smc = {})
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
        patrol_pitch_amplitude_rad_(patrol_pitch_amplitude_rad),
        patrol_pitch_angular_rate_rad_s_(patrol_pitch_angular_rate_rad_s),
        patrol_yaw_rate_rad_s_(patrol_yaw_rate_rad_s),
        reverse_flag_(reverse_flag ? 1.0f : -1.0f),
        rotor_ff_enabled_(rotor_ff_enabled),
        yaw_manual_controller_(yaw_manual_controller),
        yaw_ai_controller_(yaw_ai_controller),
        yaw_lqr_eso_config_(yaw_lqr_eso),
        yaw_smc_config_(yaw_smc),
        chassis_motion_state_topic_(
            LibXR::Topic::FindOrCreate<ChassisMotionState>(
                CHASSIS_MOTION_STATE_TOPIC_NAME, nullptr,
                CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER)) {
    UNUSED(app);

    thread_.Create(this, ThreadFunc, "GimbalThread", task_stack_depth,
                   thread_priority);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->RequestMode(GimbalEvent::SET_MODE_RELAX);
          gimbal->SetVisionTask(GimbalEvent::SET_VISION_IDLE);
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          gimbal->RequestMode(GimbalEvent::SET_MODE_RELAX);
          gimbal->SetVisionTask(GimbalEvent::SET_VISION_IDLE);
        },
        this);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->RequestMode(static_cast<GimbalEvent>(event_id));
        },
        this);
    auto vision_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          UNUSED(in_isr);
          gimbal->SetVisionTask(static_cast<GimbalEvent>(event_id));
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
    for (uint32_t event = static_cast<uint32_t>(GimbalEvent::SET_VISION_IDLE);
         event <= static_cast<uint32_t>(GimbalEvent::SET_VISION_BIG_BUFF);
         ++event) {
      gimbal_event_.Register(event, vision_callback);
    }
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
        chassis_motion_state_suber(
            LibXR::Topic(gimbal->chassis_motion_state_topic_));
    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();
    chassis_motion_state_suber.StartWaiting();

    gimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true) {
      gimbal->ConsumeModeRequests();
      if (cmd_suber.Available()) {
        gimbal->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (euler_suber.Available()) {
        const LibXR::MicrosecondTimestamp EULER_SAMPLE_TIMESTAMP =
            euler_suber.GetTimestamp();
        auto euler_sample = euler_suber.GetData();
        if (GimbalInputGuard::AllFinite({euler_sample.Roll(),
                                         euler_sample.Pitch(),
                                         euler_sample.Yaw()})) {
          euler_sample.Pitch() *= -1.0f;
          gimbal->euler_ = euler_sample;
          gimbal->last_euler_rx_time_ = EULER_SAMPLE_TIMESTAMP;
          gimbal->euler_received_ = true;
        } else {
          gimbal->euler_received_ = false;
        }
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        const LibXR::MicrosecondTimestamp GYRO_SAMPLE_TIMESTAMP =
            gyro_suber.GetTimestamp();
        auto gyro_sample = gyro_suber.GetData();
        if (GimbalInputGuard::AllFinite(
                {gyro_sample.x(), gyro_sample.y(), gyro_sample.z()})) {
          gyro_sample.y() *= -1.0f;
          gimbal->gyro_data_ = gyro_sample;
          gimbal->last_gyro_rx_time_ = GYRO_SAMPLE_TIMESTAMP;
          gimbal->gyro_received_ = true;
        } else {
          gimbal->gyro_received_ = false;
        }
        gyro_suber.StartWaiting();
      }
      if (chassis_motion_state_suber.Available()) {
        gimbal->chassis_motion_state_ = chassis_motion_state_suber.GetData();
        chassis_motion_state_suber.StartWaiting();
      }

      gimbal->Update();
      const LibXR::MicrosecondTimestamp NOW =
          LibXR::Timebase::GetMicroseconds();
      const bool IMU_VALID =
          gimbal->euler_received_ && gimbal->gyro_received_ &&
          (NOW - gimbal->last_euler_rx_time_).ToMicrosecond() <=
              IMU_TIMEOUT_US &&
          (NOW - gimbal->last_gyro_rx_time_).ToMicrosecond() <=
              IMU_TIMEOUT_US &&
          GimbalInputGuard::AllFinite(
              {gimbal->euler_.Roll(), gimbal->euler_.Pitch(),
               gimbal->euler_.Yaw(), gimbal->gyro_data_.x(),
               gimbal->gyro_data_.y(), gimbal->gyro_data_.z()});
      gimbal->imu_input_valid_ = IMU_VALID;
      const bool INPUTS_VALID = gimbal->motor_feedback_online_ && IMU_VALID;
      GimbalInputGuard::UpdateFaultLatch(INPUTS_VALID,
                                         gimbal->input_fault_latched_);
      gimbal->UpdateFreshEpoch(INPUTS_VALID);
      gimbal->ApplyConsumedModeRequest(INPUTS_VALID);
      if (!GimbalInputGuard::ControlAllowed(INPUTS_VALID,
                                            gimbal->input_fault_latched_)) {
        if (!INPUTS_VALID) {
          gimbal->RequestMode(GimbalEvent::SET_MODE_RELAX);
          gimbal->ApplyMode(GimbalEvent::SET_MODE_RELAX);
        }
        gimbal->Control();
        LibXR::Thread::Sleep(2);
        continue;
      }
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
    uint8_t mode = static_cast<uint8_t>(current_mode_);
    topic_mode_.Publish(mode);
    topic_vision_task_.Publish(vision_task_);
  }

  /**
   * @brief 解析云台控制命令
   */
  void ParseCMD() {
    const auto CTRL_MODE = cmd_.GetCtrlMode();
    const bool AI_GIMBAL_ACTIVE = cmd_.GetAIGimbalStatus();
    const bool OPERATOR_CONTROL = CTRL_MODE == CMD::Mode::CMD_OP_CTRL;
    const bool LOW_SENSITIVITY =
        current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY;
    const bool AUTOPATROL = current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL;
    const bool AI_YAW_ACTIVE =
        CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && AI_GIMBAL_ACTIVE;
    ai_yaw_active_ = AI_YAW_ACTIVE;

    if (AI_YAW_ACTIVE) {
      target_pit_cmd_ = cmd_data_.pit;
      target_pit_dot_ = cmd_data_.pit_dot;
      target_pit_ddot_ = cmd_data_.pit_ddot;
    } else if (!OPERATOR_CONTROL && AUTOPATROL) {
      const float ELAPSED_S =
          static_cast<float>(
              (LibXR::Timebase::GetMilliseconds() - patrol_start_time_)
                  .ToMillisecond()) /
          1000.0f;
      target_pit_cmd_ = PatrolTrajectory::PitchTarget(
          patrol_pitch_center_rad_, patrol_pitch_amplitude_rad_,
          patrol_pitch_angular_rate_rad_s_, ELAPSED_S);
      target_pit_dot_ = 0.0f;
      target_pit_ddot_ = 0.0f;
    } else {
      const float PITCH_SENSITIVITY =
          OPERATOR_CONTROL && LOW_SENSITIVITY ? 0.1f : 1.0f;
      const float PIT_OPERATOR_RATE =
          cmd_data_.pit * GIMBAL_MAX_SPEED * PITCH_SENSITIVITY;
      target_pit_cmd_ += PIT_OPERATOR_RATE * dt_;
      target_pit_dot_ = PIT_OPERATOR_RATE;
      target_pit_ddot_ = 0.0f;
    }

    if (AI_YAW_ACTIVE) {
      return;
    }

    if (OPERATOR_CONTROL) {
      const float YAW_SENSITIVITY = LOW_SENSITIVITY ? 0.1f : 1.0f;
      const float YAW_OPERATOR_RATE =
          cmd_data_.yaw * GIMBAL_MAX_SPEED * YAW_SENSITIVITY;
      target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
      target_yaw_dot_ = YAW_OPERATOR_RATE;
    } else if (AUTOPATROL) {
      target_yaw_cmd_ += patrol_yaw_rate_rad_s_ * dt_;
      target_yaw_dot_ = patrol_yaw_rate_rad_s_;
    } else {
      const float YAW_OPERATOR_RATE = -cmd_data_.yaw * GIMBAL_MAX_SPEED;
      target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
      target_yaw_dot_ = YAW_OPERATOR_RATE;
    }
    target_yaw_ddot_ = 0.0f;
  }

  /**
   * @brief 云台控制计算与输出
   */
  void Control() {
    const bool INPUTS_VALID = motor_feedback_online_ && imu_input_valid_;
    GimbalInputGuard::UpdateFaultLatch(INPUTS_VALID, input_fault_latched_);
    if (!GimbalInputGuard::ControlAllowed(INPUTS_VALID, input_fault_latched_)) {
      // 反馈无效时立即切松弛，避免继续使用旧反馈闭环输出。
      if (!INPUTS_VALID) {
        RequestMode(GimbalEvent::SET_MODE_RELAX);
        ApplyMode(GimbalEvent::SET_MODE_RELAX);
      }
      SubmitRelaxOutput();
      return;
    }

    float pit_output = 0.0f;
    float yaw_output = 0.0f;

    if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
      SubmitRelaxOutput();
      return;
    }

    PitchLimit(target_pit_cmd_, euler_.Pitch(), motor_pit_feedback_.abs_angle,
               pit_max_angle_, pit_min_angle_, reverse_flag_);
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
  bool motor_feedback_online_ = true;
  bool imu_input_valid_ = false;
  std::atomic_bool input_fault_latched_{true};

  CMD::GimbalCMD cmd_data_;
  Eigen::Matrix<float, 3, 1> gyro_data_;
  LibXR::EulerAngle<float> euler_;
  LibXR::MicrosecondTimestamp last_euler_rx_time_;
  LibXR::MicrosecondTimestamp last_gyro_rx_time_;
  bool euler_received_ = false;
  bool gyro_received_ = false;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;

  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_pit_angle_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_angle");
  LibXR::Topic topic_mode_ = LibXR::Topic::CreateTopic<uint8_t>("gimbal_mode");
  LibXR::Topic topic_vision_task_ =
      LibXR::Topic::CreateTopic<uint8_t>("vision_task");
  uint8_t vision_task_ = 0U;

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
  float patrol_pitch_amplitude_rad_ = 0.0f;
  float patrol_pitch_angular_rate_rad_s_ = 0.0f;
  float patrol_yaw_rate_rad_s_ = 0.0f;
  float patrol_pitch_center_rad_ = 0.0f;
  float target_pit_cmd_ = 0.0f;
  LibXR::CycleValue<float> target_yaw_cmd_ = 0.0f;
  float abs_angle_yaw_ = 0.0f;
  float abs_angle_pit_ = 0.0f;
  float last_pit_angle_loop_omega_ = 0.0f;
  float last_yaw_angle_loop_omega_ = 0.0f;
  float reverse_flag_ = 1.0f;
  LibXR::MillisecondTimestamp patrol_start_time_ = 0.0f;
  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_;
  bool rotor_ff_enabled_ = false;
  YawManualController yaw_manual_controller_ = YawManualController::PID;
  YawAiController yaw_ai_controller_ = YawAiController::LQR_ESO;
  YawLqrEso::Config yaw_lqr_eso_config_{};
  YawLqrEso yaw_lqr_eso_{};
  YawSmc::Config yaw_smc_config_{};
  YawSmc yaw_smc_{};
  bool ai_yaw_active_ = false;
  bool yaw_lqr_eso_reset_pending_ = true;
  bool yaw_smc_reset_pending_ = true;
  bool previous_smc_ai_yaw_active_ = false;
  bool previous_yaw_used_smc_ = false;
  bool previous_ai_used_lqr_ = false;
  float last_submitted_yaw_torque_nm_ = 0.0f;
  bool last_submitted_yaw_torque_valid_ = false;
  ChassisMotionState chassis_motion_state_{};
  LibXR::Topic::TopicHandle chassis_motion_state_topic_;
  LibXR::MPMCQueue<GimbalModeRequest> mode_requests_{4};
  std::atomic<uint32_t> request_sequence_{0U};
  std::atomic<uint32_t> relax_sequence_{0U};
  std::atomic<uint32_t> fresh_epoch_{0U};
  GimbalInputGuard::ModeProtocol mode_protocol_;
  GimbalModeRequest pending_mode_request_{GimbalEvent::SET_MODE_RELAX, 0U, 0U};
  bool pending_mode_request_valid_ = false;
  bool pending_relax_request_ = false;
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

  void SubmitRelaxOutput() {
    pid_pit_omega_.SetFeedForward(0.0f);
    pid_yaw_omega_.SetFeedForward(0.0f);
    last_pit_angle_loop_omega_ = 0.0f;
    last_yaw_angle_loop_omega_ = 0.0f;
    motor_yaw_->Relax();
    motor_pit_->Relax();
  }

  void RequestMode(GimbalEvent gimbal_event) {
    const uint32_t FRESH_EPOCH = fresh_epoch_.load(std::memory_order_acquire);
    const uint32_t REQUEST_SEQUENCE = NextRequestSequence();
    if (gimbal_event == GimbalEvent::SET_MODE_RELAX) {
      PublishRelaxSequence(REQUEST_SEQUENCE);
      return;
    }

    GimbalModeRequest request{gimbal_event, REQUEST_SEQUENCE, FRESH_EPOCH};
    const auto PUSH_RESULT = mode_requests_.Push(request);
    if (PUSH_RESULT == LibXR::ErrorCode::FULL) {
      GimbalModeRequest discarded_request;
      (void)mode_requests_.Pop(discarded_request);
      (void)mode_requests_.Push(request);
    }
  }

  void ConsumeModeRequests() {
    ConsumeRelaxSequence();

    GimbalModeRequest latest_request = pending_mode_request_;
    bool mode_request_available =
        pending_mode_request_valid_ &&
        mode_protocol_.OrdinaryIsCurrent(latest_request.sequence);
    pending_mode_request_valid_ = false;

    GimbalModeRequest request;
    while (mode_requests_.Pop(request) == LibXR::ErrorCode::OK) {
      if (!mode_protocol_.ConsumeOrdinary(request.sequence)) {
        continue;
      }
      latest_request = request;
      mode_request_available = true;
    }

    ConsumeRelaxSequence();
    if (mode_request_available &&
        !mode_protocol_.OrdinaryIsCurrent(latest_request.sequence)) {
      mode_request_available = false;
    }

    pending_mode_request_ = latest_request;
    pending_mode_request_valid_ = mode_request_available;
  }

  void ApplyConsumedModeRequest(bool inputs_valid) {
    ConsumeRelaxSequence();
    if (pending_mode_request_valid_ &&
        !mode_protocol_.OrdinaryIsCurrent(pending_mode_request_.sequence)) {
      pending_mode_request_valid_ = false;
    }
    if (pending_relax_request_) {
      pending_relax_request_ = false;
      ApplyMode(GimbalEvent::SET_MODE_RELAX);
      return;
    }
    if (!pending_mode_request_valid_) {
      return;
    }

    pending_mode_request_valid_ = false;
    if (!mode_protocol_.CanApplyOrdinary(pending_mode_request_.sequence,
                                         pending_mode_request_.fresh_epoch,
                                         inputs_valid)) {
      return;
    }
    if (!GimbalInputGuard::AcceptActiveRequest(inputs_valid,
                                               input_fault_latched_)) {
      return;
    }
    ApplyMode(pending_mode_request_.mode);
    mode_protocol_.RecordOrdinaryApplied(pending_mode_request_.sequence);
  }

  uint32_t NextRequestSequence() {
    uint32_t current = request_sequence_.load(std::memory_order_relaxed);
    while (true) {
      uint32_t next = current + 1U;
      if (next == 0U) {
        next = 1U;
      }
      if (request_sequence_.compare_exchange_weak(current, next,
                                                  std::memory_order_relaxed,
                                                  std::memory_order_relaxed)) {
        return next;
      }
    }
  }

  void PublishRelaxSequence(uint32_t sequence) {
    uint32_t current = relax_sequence_.load(std::memory_order_relaxed);
    while ((current == 0U ||
            GimbalInputGuard::IsSequenceAfter(sequence, current)) &&
           !relax_sequence_.compare_exchange_weak(current, sequence,
                                                  std::memory_order_release,
                                                  std::memory_order_relaxed)) {
    }
  }

  void ConsumeRelaxSequence() {
    const uint32_t SEQUENCE =
        relax_sequence_.exchange(0U, std::memory_order_acq_rel);
    if (SEQUENCE == 0U || !mode_protocol_.ConsumeRelax(SEQUENCE)) {
      return;
    }
    pending_relax_request_ = true;
  }

  void UpdateFreshEpoch(bool inputs_valid) {
    fresh_epoch_.store(mode_protocol_.ObserveInputs(inputs_valid),
                       std::memory_order_release);
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
      yaw_smc_.CommitAppliedTorque(command.torque);
    }
  }

  /**
   * @brief 解算PID控制输出
   */
  void Solve(float& pit_output, float& yaw_output) {
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
    pit_output =
        pid_pit_omega_.Calculate(TARGET_PIT_OMEGA, gyro_data_.y(), dt_);
    last_pit_angle_loop_omega_ = PIT_ANGLE_LOOP_OMEGA;

    const bool NEXT_USES_SMC =
        ai_yaw_active_ ? yaw_ai_controller_ == YawAiController::SMC
                       : yaw_manual_controller_ == YawManualController::SMC;
    if (NEXT_USES_SMC && (ai_yaw_active_ != previous_smc_ai_yaw_active_ ||
                          !previous_yaw_used_smc_)) {
      yaw_smc_reset_pending_ = true;
    }
    if (ai_yaw_active_ && yaw_ai_controller_ == YawAiController::LQR_ESO &&
        !previous_ai_used_lqr_) {
      yaw_lqr_eso_reset_pending_ = true;
    }
    previous_smc_ai_yaw_active_ = ai_yaw_active_;
    previous_yaw_used_smc_ = NEXT_USES_SMC;
    previous_ai_used_lqr_ =
        ai_yaw_active_ && yaw_ai_controller_ == YawAiController::LQR_ESO;

    if (ai_yaw_active_) {
      SolveAiYaw(yaw_output);
    } else if (yaw_manual_controller_ == YawManualController::SMC) {
      SolveManualYawSmc(yaw_output);
    } else {
      SolveLegacyYaw(yaw_output);
    }
  }

  void SolveLegacyYaw(float& yaw_output) {
    const float YAW_ERROR = target_yaw_cmd_ - euler_.Yaw();
    const float YAW_ANGLE_LOOP_OMEGA =
        pid_yaw_angle_.Calculate(YAW_ERROR, 0.0f, dt_);
    const float TARGET_YAW_OMEGA = YAW_ANGLE_LOOP_OMEGA + target_yaw_dot_;
    const float YAW_ALPHA =
        (YAW_ANGLE_LOOP_OMEGA - last_yaw_angle_loop_omega_) / dt_ +
        target_yaw_ddot_;
    const bool ROTOR_FF_ACTIVE =
        rotor_ff_enabled_ && chassis_motion_state_.online &&
        chassis_motion_state_.yaw_rate_valid &&
        chassis_motion_state_.mode == ChassisMotionMode::ROTOR;
    const float YAW_MOTOR_OMEGA_REF =
        ROTOR_FF_ACTIVE
            ? TARGET_YAW_OMEGA - chassis_motion_state_.yaw_rate_rad_s
            : TARGET_YAW_OMEGA;
    const float YAW_FEEDFORWARD =
        j_yaw_ * YAW_ALPHA + yaw_k_ * YAW_MOTOR_OMEGA_REF;
    pid_yaw_omega_.SetFeedForward(YAW_FEEDFORWARD);
    yaw_output =
        pid_yaw_omega_.Calculate(TARGET_YAW_OMEGA, gyro_data_.z(), dt_);
    last_yaw_angle_loop_omega_ = YAW_ANGLE_LOOP_OMEGA;
  }

  void SolveAiYaw(float& yaw_output) {
    if (yaw_ai_controller_ == YawAiController::SMC) {
      SolveAiYawSmc(yaw_output);
    } else {
      SolveAiYawLqrEso(yaw_output);
    }
  }

  void SolveAiYawLqrEso(float& yaw_output) {
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
        dt_, j_yaw_, pid_yaw_omega_.OutLimit());
    if (!YAW_LQR_ESO_OUTPUT.valid ||
        !std::isfinite(YAW_LQR_ESO_OUTPUT.tau_cmd_nm)) {
      yaw_output = 0.0f;
      yaw_lqr_eso_reset_pending_ = true;
      return;
    }
    yaw_lqr_eso_reset_pending_ = false;
    yaw_output = YAW_LQR_ESO_OUTPUT.tau_cmd_nm;
  }

  void SolveAiYawSmc(float& yaw_output) {
    SolveSmcYaw(cmd_data_.yaw, cmd_data_.yaw_dot, cmd_data_.yaw_ddot,
                yaw_output);
  }

  void SolveManualYawSmc(float& yaw_output) {
    SolveSmcYaw(static_cast<float>(target_yaw_cmd_), target_yaw_dot_,
                target_yaw_ddot_, yaw_output);
  }

  void SolveSmcYaw(float theta_ref, float omega_ref, float alpha_ref,
                   float& yaw_output) {
    if (yaw_smc_reset_pending_) {
      const float PREVIOUS_TORQUE = last_submitted_yaw_torque_valid_
                                        ? last_submitted_yaw_torque_nm_
                                        : 0.0f;
      yaw_smc_.Reset(euler_.Yaw(), gyro_data_.z(), PREVIOUS_TORQUE);
    }
    const auto YAW_SMC_OUTPUT =
        yaw_smc_.Calculate(yaw_smc_config_,
                           {.theta_rad = theta_ref,
                            .omega_rad_s = omega_ref,
                            .alpha_rad_s2 = alpha_ref},
                           {.theta_rad = euler_.Yaw(),
                            .omega_rad_s = gyro_data_.z(),
                            .valid = motor_feedback_online_},
                           dt_);
    if (!YAW_SMC_OUTPUT.valid || !std::isfinite(YAW_SMC_OUTPUT.tau_cmd_nm)) {
      yaw_output = 0.0f;
      yaw_smc_reset_pending_ = true;
      return;
    }
    yaw_smc_reset_pending_ = false;
    yaw_output = YAW_SMC_OUTPUT.tau_cmd_nm;
  }

  /**
   * @brief 设置云台模式
   *
   * @param gimbal_event 云台事件类型
   */
  void ApplyMode(GimbalEvent gimbal_event) {
    if (gimbal_event == current_mode_) {
      return;
    }
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
    last_pit_angle_loop_omega_ = 0.0f;
    last_yaw_angle_loop_omega_ = 0.0f;
    yaw_smc_reset_pending_ = true;
    yaw_lqr_eso_reset_pending_ = true;
    previous_yaw_used_smc_ = false;
    previous_ai_used_lqr_ = false;

    const bool RELAX = gimbal_event == GimbalEvent::SET_MODE_RELAX;
    const bool TRACKING_MODE =
        gimbal_event == GimbalEvent::SET_MODE_COMMON ||
        gimbal_event == GimbalEvent::SET_MODE_AUTOPATROL ||
        gimbal_event == GimbalEvent::SET_MODE_LOW_SENSITIVITY;
    if (!RELAX && !TRACKING_MODE) {
      return;
    }

    pid_pit_angle_.Reset();
    pid_pit_omega_.Reset();
    pid_yaw_angle_.Reset();
    pid_yaw_omega_.Reset();
    target_yaw_dot_ = 0.0f;
    target_yaw_ddot_ = 0.0f;
    target_pit_dot_ = 0.0f;
    target_pit_ddot_ = 0.0f;

    if (RELAX) {
      motor_yaw_->Disable();
      motor_pit_->Disable();
      target_pit_cmd_ = 0.0f;
      target_yaw_cmd_ = 0.0f;
      return;
    }

    target_pit_cmd_ = euler_.Pitch();
    target_yaw_cmd_ = euler_.Yaw();
    if (gimbal_event == GimbalEvent::SET_MODE_AUTOPATROL) {
      patrol_pitch_center_rad_ = target_pit_cmd_;
      patrol_start_time_ = LibXR::Timebase::GetMilliseconds();
    }
  }

  void SetVisionTask(GimbalEvent event) {
    const auto value = static_cast<uint8_t>(event);
    const auto first = static_cast<uint8_t>(GimbalEvent::SET_VISION_IDLE);
    const auto last = static_cast<uint8_t>(GimbalEvent::SET_VISION_BIG_BUFF);
    if (value < first || value > last) return;
    vision_task_ = static_cast<uint8_t>(value - first);
  }
};

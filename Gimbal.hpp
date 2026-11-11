#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Two-axis gimbal controller with vector inverse kinematics
depends:
- id: GUET-PLDX/CMD
- id: GUET-PLDX/Motor
- id: xrobot-org/BMI088
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>

#include "CMD.hpp"
#include "Motor.hpp"
#include "SystemIdentify.hpp"
#include "TwoAxisIK.hpp"
#include "YawSmc.hpp"
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
    YawSmc::Param yaw_smc;
    YawSmc::Param syaw_smc;
  };

  struct YawFollowParam {
    bool enabled;
    float deadband_rad;
  };

  struct Param {
    uint32_t task_stack_depth;
    ControlParam manual;
    ControlParam auto_aim;
    YawSmc::OutputLimits yaw_output_limits;
    TwoAxisIK::Config two_axis_ik;
    YawFollowParam yaw_follow;
    float pit_lc;
    float pit_theta;
    float j_pit;
    float j_yaw;
    float j_syaw;
    float yaw_zero;
    float imu_timeout_s;
    float patrol_pitch_amplitude_rad;
    float patrol_pitch_angular_rate_rad_s;
    float patrol_yaw_rate_rad_s;
    float patrol_imu_timeout_s;
    LibXR::Thread::Priority thread_priority;
    SystemIdentify::Config system_identify;
    const char *euler_topic_name;
    const char *gyro_topic_name;
    const char *gimbal_cmd_topic_name;
  };

  Gimbal(CMD &cmd, Motor &motor_pit, Motor &motor_yaw, Motor &motor_syaw,
         Motor &motor_fold,
         const Param &param =
             Param{.task_stack_depth = 2048,
                   .manual = {},
                   .auto_aim = {},
                   .yaw_output_limits = {},
                   .two_axis_ik = {},
                   .yaw_follow = {.enabled = false, .deadband_rad = 0.0f},
                   .pit_lc = 0.0f,
                   .pit_theta = 0.0f,
                   .j_pit = 0.0f,
                   .j_yaw = 0.0f,
                   .j_syaw = 0.0f,
                   .yaw_zero = 0.0f,
                   .imu_timeout_s = 0.05f,
                   .patrol_pitch_amplitude_rad = 0.0f,
                   .patrol_pitch_angular_rate_rad_s = 0.0f,
                   .patrol_yaw_rate_rad_s = 0.0f,
                   .patrol_imu_timeout_s = 0.05f,
                   .thread_priority = LibXR::Thread::Priority::MEDIUM,
                   .system_identify = {},
                   .euler_topic_name = "gimbal_euler",
                   .gyro_topic_name = "gimbal_gyro",
                   .gimbal_cmd_topic_name = "gimbal_cmd"})
      : cmd_(cmd), PARAM(param),
        manual_control_(PARAM.manual, PARAM.yaw_output_limits),
        auto_aim_control_(PARAM.auto_aim, PARAM.yaw_output_limits),
        motor_yaw_(&motor_yaw), motor_pit_(&motor_pit),
        motor_syaw_(&motor_syaw), motor_fold_(&motor_fold),
        system_identify_(param.system_identify) {
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t) {
          UNUSED(in_isr);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal *gimbal, uint32_t event_id) {
          ASSERT(!in_isr);
          if (in_isr)
            return;
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
  }

  static void ThreadFunc(Gimbal *gimbal) {
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
        auto sample = euler_suber.GetData();
        gimbal->euler_timestamp_ = euler_suber.GetTimestamp();
        gimbal->euler_received_ = true;
        sample.Pitch() *= -1.0f;
        gimbal->euler_ = sample;
        euler_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        auto sample = gyro_suber.GetData();
        gimbal->gyro_timestamp_ = gyro_suber.GetTimestamp();
        gimbal->gyro_received_ = true;
        sample.y() *= -1.0f;
        gimbal->gyro_data_ = sample;
        gyro_suber.StartWaiting();
      }
      gimbal->Update();
      gimbal->ParseCMD();
      gimbal->Control();
      LibXR::Thread::SleepUntil(last_time, 1);
    }
  }

  void Update() {
    if (!identify_boot_checked_) {
      identify_boot_checked_ = true;
      if (system_identify_.GetConfig().enabled &&
          current_mode_ == GimbalEvent::SET_MODE_RELAX) {
        SetMode(GimbalEvent::SET_MODE_IDENTIFY);
      }
    }
    motor_yaw_->Update();
    motor_pit_->Update();
    motor_syaw_->Update();
    motor_fold_->Update();
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_pit_feedback_ = motor_pit_->GetFeedback();
    motor_syaw_feedback_ = motor_syaw_->GetFeedback();
    motor_fold_feedback_ = motor_fold_->GetFeedback();
    const auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    float big_yaw = motor_yaw_feedback_.abs_angle - PARAM.yaw_zero;
    float pitch =
        JointPosition(motor_pit_feedback_, PARAM.two_axis_ik.pitch_sign,
                      PARAM.two_axis_ik.pitch_zero_rad);
    float small_yaw =
        JointPosition(motor_syaw_feedback_, PARAM.two_axis_ik.yaw_sign,
                      PARAM.two_axis_ik.yaw_zero_rad);
    topic_yaw_angle_.Publish(big_yaw);
    topic_pit_angle_.Publish(pitch);
    topic_syaw_angle_.Publish(small_yaw);
    topic_yaw_omega_.Publish(motor_yaw_feedback_.omega);
    topic_pit_omega_.Publish(motor_pit_feedback_.omega);
    topic_syaw_omega_.Publish(motor_syaw_feedback_.omega);
    uint8_t mode = static_cast<uint8_t>(current_mode_);
    topic_mode_.Publish(mode);
  }

  void ParseCMD() {
    if (!std::isfinite(dt_) || dt_ <= 0.0f || dt_ > 0.02f) {
      return;
    }
    if (current_mode_ != GimbalEvent::SET_MODE_RELAX &&
        (!SensorsValid() || !motor_syaw_->IsOnline() ||
         !motor_pit_->IsOnline() || motor_syaw_feedback_.state != 1 ||
         motor_pit_feedback_.state != 1 || !motor_yaw_->IsOnline() ||
         motor_yaw_feedback_.state != 1)) {
      target_initialized_ = false;
      if (euler_received_) {
        SyncTargetToFeedback();
      }
      return;
    }
    if (current_mode_ != GimbalEvent::SET_MODE_RELAX && !target_initialized_) {
      SyncTargetToFeedback();
      target_initialized_ = true;
      return;
    }
    const auto ctrl_mode = cmd_.GetCtrlMode();
    const bool ai_active =
        ctrl_mode == CMD::Mode::CMD_AUTO_CTRL && cmd_.GetAIGimbalStatus() &&
        (current_mode_ == GimbalEvent::SET_VISION_AUTO_AIM ||
         current_mode_ == GimbalEvent::SET_VISION_SMALL_BUFF ||
         current_mode_ == GimbalEvent::SET_VISION_BIG_BUFF);
    if (ai_active != ai_gimbal_reference_active_) {
      ai_gimbal_reference_active_ = ai_active;
      ActiveControl().Reset(last_applied_yaw_torque_nm_,
                            last_applied_syaw_torque_nm_);
      SyncTargetToFeedback();
      target_yaw_rate_ref_ = 0.0f;
      target_yaw_accel_ref_ = 0.0f;
      control_profile_just_changed_ = true;
    }
    if (control_profile_just_changed_) {
      control_profile_just_changed_ = false;
      return;
    }
    if (ctrl_mode == CMD::Mode::CMD_OP_CTRL) {
      target_world_.Yaw() += cmd_data_.yaw * dt_ * GIMBAL_MAX_SPEED;
      target_world_.Pitch() += cmd_data_.pit * dt_ * GIMBAL_MAX_SPEED;
      target_yaw_rate_ref_ = 0.0f;
      target_yaw_accel_ref_ = 0.0f;
      return;
    }
    if (ai_gimbal_reference_active_) {
      target_world_.Yaw() = cmd_data_.yaw;
      target_world_.Pitch() = cmd_data_.pit;
      target_world_.Roll() = cmd_data_.rol;
      target_yaw_rate_ref_ = cmd_data_.yaw_dot;
      target_yaw_accel_ref_ = cmd_data_.yaw_ddot;
      return;
    }
    if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
      UpdatePatrol();
      return;
    }
    target_world_.Yaw() -= cmd_data_.yaw * dt_ * GIMBAL_MAX_SPEED;
    target_world_.Pitch() += cmd_data_.pit * dt_ * GIMBAL_MAX_SPEED;
    target_yaw_rate_ref_ = 0.0f;
    target_yaw_accel_ref_ = 0.0f;
  }

  void Control() {
    if (current_mode_ == GimbalEvent::SET_MODE_RELAX) {
      RelaxAll();
      return;
    }
    if (current_mode_ == GimbalEvent::SET_MODE_IDENTIFY) {
      if (PARAM.system_identify.enabled) {
        ControlIdentify();
      } else {
        RelaxAll();
      }
      return;
    }
    if (!MotorsReady()) {
      ResetControls();
      return;
    }
    if (!SensorsValid()) {
      RelaxAll();
      return;
    }

    const float measured_pitch =
        JointPosition(motor_pit_feedback_, PARAM.two_axis_ik.pitch_sign,
                      PARAM.two_axis_ik.pitch_zero_rad);
    const float measured_yaw_rate =
        PARAM.two_axis_ik.yaw_sign * motor_syaw_feedback_.omega;
    const float measured_pitch_rate =
        PARAM.two_axis_ik.pitch_sign * motor_pit_feedback_.omega;
    const auto ik =
        TwoAxisIK::Solve(target_world_, euler_, motor_syaw_feedback_.abs_angle,
                         motor_pit_feedback_.abs_angle, PARAM.two_axis_ik);
    if (!ik.valid) {
      RelaxAll();
      return;
    }

    const float yaw_joint_target = JointPosition(
        ik.yaw_rad, PARAM.two_axis_ik.yaw_sign, PARAM.two_axis_ik.yaw_zero_rad);
    const float pitch_joint_target =
        JointPosition(ik.pitch_rad, PARAM.two_axis_ik.pitch_sign,
                      PARAM.two_axis_ik.pitch_zero_rad);

    auto &control = ActiveControl();
    const float measured_yaw =
        JointPosition(motor_syaw_feedback_, PARAM.two_axis_ik.yaw_sign,
                      PARAM.two_axis_ik.yaw_zero_rad);
    const float syaw_rate_ref =
        JointReferenceRate(yaw_joint_target, control.last_syaw_position, true,
                           control.syaw_reference_primed);
    const float syaw_accel_ref =
        control.syaw_reference_primed
            ? std::clamp((syaw_rate_ref - control.last_syaw_speed) / dt_,
                         -MAX_JOINT_REFERENCE_ACCEL, MAX_JOINT_REFERENCE_ACCEL)
            : 0.0f;
    const float pitch_speed_ref = std::clamp(
        control.pid_pit_angle.Calculate(pitch_joint_target, measured_pitch,
                                        measured_pitch_rate, dt_) +
            JointReferenceRate(pitch_joint_target, control.last_pitch_position,
                               false, control.pitch_reference_primed),
        -MAX_JOINT_SPEED_REF, MAX_JOINT_SPEED_REF);
    const float pitch_accel_ref =
        control.pitch_reference_primed
            ? std::clamp((pitch_speed_ref - control.last_pitch_speed) / dt_,
                         -MAX_JOINT_REFERENCE_ACCEL, MAX_JOINT_REFERENCE_ACCEL)
            : 0.0f;
    control.pid_pit_omega.SetFeedForward(
        PARAM.j_pit * pitch_accel_ref -
        PARAM.pit_lc * std::sin(euler_.Pitch() + PARAM.pit_theta));
    const float pitch_torque = control.pid_pit_omega.Calculate(
        pitch_speed_ref, measured_pitch_rate, dt_);

    float syaw_follow_error = measured_yaw;
    if (PARAM.yaw_follow.enabled) {
      if (!std::isfinite(PARAM.yaw_follow.deadband_rad) ||
          PARAM.yaw_follow.deadband_rad < 0.0f ||
          PARAM.yaw_follow.deadband_rad >= static_cast<float>(LibXR::PI)) {
        RelaxAll();
        return;
      }
      if (std::fabs(syaw_follow_error) <= PARAM.yaw_follow.deadband_rad) {
        syaw_follow_error = 0.0f;
      } else {
        syaw_follow_error -=
            std::copysign(PARAM.yaw_follow.deadband_rad, syaw_follow_error);
      }
    } else {
      syaw_follow_error = 0.0f;
    }
    const float big_yaw_target =
        PARAM.yaw_follow.enabled
            ? static_cast<float>(motor_yaw_feedback_.abs_angle) +
                  syaw_follow_error
            : target_world_.Yaw();
    const float big_yaw_rate =
        PARAM.yaw_follow.enabled
            ? motor_yaw_feedback_.omega +
                  (syaw_follow_error == 0.0f ? 0.0f : measured_yaw_rate)
            : target_yaw_rate_ref_;
    const float big_yaw_accel =
        PARAM.yaw_follow.enabled ? syaw_accel_ref : target_yaw_accel_ref_;
    const auto big_yaw_output = control.yaw_smc.Calculate(
        {.theta_rad = big_yaw_target,
         .omega_rad_s = big_yaw_rate,
         .alpha_rad_s2 = big_yaw_accel},
        {.theta_rad = PARAM.yaw_follow.enabled
                          ? static_cast<float>(motor_yaw_feedback_.abs_angle)
                          : euler_.Yaw(),
         .omega_rad_s = PARAM.yaw_follow.enabled ? motor_yaw_feedback_.omega
                                                 : gyro_data_.z(),
         .valid = motor_yaw_->IsOnline() && motor_yaw_feedback_.state == 1},
        dt_, PARAM.j_yaw);
    const auto small_yaw_output = control.syaw_smc.Calculate(
        {.theta_rad = yaw_joint_target,
         .omega_rad_s = syaw_rate_ref,
         .alpha_rad_s2 = syaw_accel_ref},
        {.theta_rad =
             JointPosition(motor_syaw_feedback_, PARAM.two_axis_ik.yaw_sign,
                           PARAM.two_axis_ik.yaw_zero_rad),
         .omega_rad_s = measured_yaw_rate,
         .valid = motor_syaw_->IsOnline() && motor_syaw_feedback_.state == 1},
        dt_, PARAM.j_syaw);

    control.last_pitch_speed = pitch_speed_ref;
    control.last_pitch_position = pitch_joint_target;
    control.last_syaw_position = yaw_joint_target;
    control.last_syaw_speed = syaw_rate_ref;
    control.pitch_reference_primed = true;
    control.syaw_reference_primed = true;
    if (!big_yaw_output.valid || !small_yaw_output.valid ||
        !std::isfinite(big_yaw_output.tau_cmd_nm) ||
        !std::isfinite(small_yaw_output.tau_cmd_nm) ||
        !std::isfinite(pitch_torque)) {
      RelaxAll();
      return;
    }

    ControlMotor(motor_yaw_, motor_yaw_feedback_,
                 {.mode = Motor::ControlMode::MODE_TORQUE,
                  .torque = big_yaw_output.tau_cmd_nm});
    ControlMotor(
        motor_syaw_, motor_syaw_feedback_,
        {.mode = Motor::ControlMode::MODE_TORQUE,
         .torque = PARAM.two_axis_ik.yaw_sign * small_yaw_output.tau_cmd_nm});
    ControlMotor(motor_pit_, motor_pit_feedback_,
                 {.mode = Motor::ControlMode::MODE_TORQUE,
                  .torque = PARAM.two_axis_ik.pitch_sign * pitch_torque});
    CommitAppliedBigYawTorque(big_yaw_output.tau_cmd_nm);
    CommitAppliedSmallYawTorque(small_yaw_output.tau_cmd_nm);
  }

  LibXR::Event &GetEvent() { return gimbal_event_; }

private:
  struct ControlState {
    LibXR::PID<float> pid_pit_angle;
    LibXR::PID<float> pid_pit_omega;
    YawSmc yaw_smc;
    YawSmc syaw_smc;
    float last_pitch_speed = 0.0f;
    float last_pitch_position = 0.0f;
    float last_syaw_speed = 0.0f;
    float last_syaw_position = 0.0f;
    bool pitch_reference_primed = false;
    bool syaw_reference_primed = false;

    ControlState(const ControlParam &param,
                 const YawSmc::OutputLimits &output_limits)
        : pid_pit_angle(param.pid_pit_angle),
          pid_pit_omega(param.pid_pit_omega),
          yaw_smc(param.yaw_smc, output_limits),
          syaw_smc(param.syaw_smc, output_limits) {}

    void Reset(float applied_yaw_torque, float applied_syaw_torque) {
      pid_pit_angle.Reset();
      pid_pit_omega.Reset();
      pid_pit_omega.SetFeedForward(0.0f);
      yaw_smc.Reset(applied_yaw_torque);
      syaw_smc.Reset(applied_syaw_torque);
      last_pitch_speed = 0.0f;
      last_pitch_position = 0.0f;
      last_syaw_speed = 0.0f;
      last_syaw_position = 0.0f;
      pitch_reference_primed = false;
      syaw_reference_primed = false;
    }
  };

  CMD &cmd_;
  const Param PARAM;
  ControlState manual_control_;
  ControlState auto_aim_control_;
  bool big_yaw_relaxed_ = false;
  float last_applied_yaw_torque_nm_ = 0.0f;
  float last_applied_syaw_torque_nm_ = 0.0f;
  Motor *motor_yaw_;
  Motor *motor_pit_;
  Motor *motor_syaw_;
  Motor *motor_fold_;
  Motor::Feedback motor_yaw_feedback_{};
  Motor::Feedback motor_pit_feedback_{};
  Motor::Feedback motor_syaw_feedback_{};
  Motor::Feedback motor_fold_feedback_{};
  CMD::GimbalCMD cmd_data_{};
  Eigen::Matrix<float, 3, 1> gyro_data_ = Eigen::Matrix<float, 3, 1>::Zero();
  LibXR::EulerAngle<float> euler_;
  LibXR::EulerAngle<float> target_world_;
  LibXR::MicrosecondTimestamp euler_timestamp_{}, gyro_timestamp_{};
  LibXR::MicrosecondTimestamp last_online_time_{};
  SystemIdentify system_identify_;
  bool euler_received_ = false;
  bool gyro_received_ = false;
  bool ai_gimbal_reference_active_ = false;
  bool control_profile_just_changed_ = false;
  bool target_initialized_ = false;
  float target_yaw_rate_ref_ = 0.0f;
  float target_yaw_accel_ref_ = 0.0f;
  bool identify_boot_checked_ = false;
  bool identify_relax_lock_ = false;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;
  float dt_ = 0.0f;
  float patrol_elapsed_s_ = 0.0f;
  float patrol_yaw_rate_ = 0.0f;

  static constexpr float GIMBAL_MAX_SPEED =
      static_cast<float>(LibXR::TWO_PI) * 2.0f;
  static constexpr float MAX_JOINT_SPEED_REF = 20.0f;
  static constexpr float MAX_JOINT_REFERENCE_ACCEL = 50.0f;

  LibXR::Event gimbal_event_;
  LibXR::Topic topic_yaw_angle_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_angle");
  LibXR::Topic topic_pit_angle_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_angle");
  LibXR::Topic topic_syaw_angle_ =
      LibXR::Topic::CreateTopic<float>("syawmotor_angle");
  LibXR::Topic topic_yaw_omega_ =
      LibXR::Topic::CreateTopic<float>("yawmotor_omega");
  LibXR::Topic topic_pit_omega_ =
      LibXR::Topic::CreateTopic<float>("pitchmotor_omega");
  LibXR::Topic topic_syaw_omega_ =
      LibXR::Topic::CreateTopic<float>("syawmotor_omega");
  LibXR::Topic topic_mode_ = LibXR::Topic::CreateTopic<uint8_t>("gimbal_mode");
  LibXR::Topic topic_vision_task_ =
      LibXR::Topic::CreateTopic<uint8_t>("vision_task");
  LibXR::Thread thread_;

  ControlState &ActiveControl() {
    return ai_gimbal_reference_active_ ? auto_aim_control_ : manual_control_;
  }

  static float JointPosition(const Motor::Feedback &feedback, float sign,
                             float zero) {
    return JointPosition(static_cast<float>(feedback.abs_angle), sign, zero);
  }

  static float JointPosition(float encoder_angle, float sign, float zero) {
    return sign * (LibXR::CycleValue<float>(encoder_angle) - zero);
  }

  float JointReferenceRate(float target, float previous, bool cyclic,
                           bool primed) const {
    if (!primed || !std::isfinite(target) || !std::isfinite(previous) ||
        !std::isfinite(dt_) || dt_ <= 0.0f) {
      return 0.0f;
    }
    const float delta = cyclic ? LibXR::CycleValue<float>(target) - previous
                               : target - previous;
    return std::clamp(delta / dt_, -20.0f, 20.0f);
  }

  void SyncTargetToFeedback() {
    target_world_.Yaw() = euler_.Yaw();
    target_world_.Pitch() = euler_.Pitch();
    target_world_.Roll() = euler_.Roll();
  }

  bool SensorsValid() const {
    const auto now = LibXR::Timebase::GetMicroseconds();
    return euler_received_ && gyro_received_ &&
           std::isfinite(PARAM.imu_timeout_s) && PARAM.imu_timeout_s > 0.0f &&
           (now - euler_timestamp_).ToSecondf() <= PARAM.imu_timeout_s &&
           (now - gyro_timestamp_).ToSecondf() <= PARAM.imu_timeout_s &&
           std::isfinite(dt_) && dt_ > 0.0005f && dt_ <= 0.02f &&
           std::isfinite(euler_.Roll()) && std::isfinite(euler_.Pitch()) &&
           std::isfinite(euler_.Yaw()) && std::isfinite(gyro_data_.x()) &&
           std::isfinite(gyro_data_.y()) && std::isfinite(gyro_data_.z()) &&
           std::isfinite(static_cast<float>(motor_pit_feedback_.abs_angle)) &&
           std::isfinite(static_cast<float>(motor_syaw_feedback_.abs_angle)) &&
           std::isfinite(motor_pit_feedback_.omega) &&
           std::isfinite(motor_syaw_feedback_.omega);
  }

  void RelaxAll() {
    RelaxBigYaw();
    motor_pit_->Relax();
    motor_syaw_->Relax();
    manual_control_.Reset(0.0f, 0.0f);
    auto_aim_control_.Reset(0.0f, 0.0f);
    last_applied_yaw_torque_nm_ = 0.0f;
    last_applied_syaw_torque_nm_ = 0.0f;
  }

  void ResetControls() {
    manual_control_.Reset(last_applied_yaw_torque_nm_,
                          last_applied_syaw_torque_nm_);
    auto_aim_control_.Reset(last_applied_yaw_torque_nm_,
                            last_applied_syaw_torque_nm_);
  }

  bool MotorsReady() {
    const Motor::MotorCmd zero_torque{.mode = Motor::ControlMode::MODE_TORQUE};
    ControlMotor(motor_yaw_, motor_yaw_feedback_, zero_torque);
    ControlMotor(motor_syaw_, motor_syaw_feedback_, zero_torque);
    ControlMotor(motor_pit_, motor_pit_feedback_, zero_torque);
    return motor_syaw_->IsOnline() && motor_pit_->IsOnline() &&
           motor_syaw_feedback_.state == 1 && motor_pit_feedback_.state == 1 &&
           motor_yaw_->IsOnline() && motor_yaw_feedback_.state == 1;
  }

  void ControlMotor(Motor *motor, const Motor::Feedback &feedback,
                    const Motor::MotorCmd &command) {
    if (feedback.state == 0) {
      motor->Enable();
      motor->Control({.mode = Motor::ControlMode::MODE_TORQUE});
    } else if (feedback.state != 1) {
      motor->ClearError();
      motor->Control({.mode = Motor::ControlMode::MODE_TORQUE});
    } else {
      motor->Control(command);
    }
    if (motor == motor_yaw_) {
      big_yaw_relaxed_ = false;
    }
  }

  void RelaxBigYaw() {
    if (!big_yaw_relaxed_) {
      motor_yaw_->Relax();
      big_yaw_relaxed_ = true;
    }
  }

  void CommitAppliedBigYawTorque(float torque) {
    if (!std::isfinite(torque))
      return;
    last_applied_yaw_torque_nm_ = torque;
    ActiveControl().yaw_smc.CommitAppliedTorque(torque);
  }

  void CommitAppliedSmallYawTorque(float torque) {
    if (!std::isfinite(torque))
      return;
    last_applied_syaw_torque_nm_ = torque;
    ActiveControl().syaw_smc.CommitAppliedTorque(torque);
  }

  void UpdatePatrol() {
    if (!std::isfinite(PARAM.patrol_imu_timeout_s) ||
        PARAM.patrol_imu_timeout_s <= 0.0f || !euler_received_ ||
        (LibXR::Timebase::GetMicroseconds() - euler_timestamp_).ToSecondf() >
            PARAM.patrol_imu_timeout_s ||
        !std::isfinite(PARAM.patrol_pitch_amplitude_rad) ||
        !std::isfinite(PARAM.patrol_pitch_angular_rate_rad_s) ||
        !std::isfinite(PARAM.patrol_yaw_rate_rad_s)) {
      SyncTargetToFeedback();
      patrol_elapsed_s_ = 0.0f;
      target_yaw_rate_ref_ = 0.0f;
      target_yaw_accel_ref_ = 0.0f;
      return;
    }
    patrol_elapsed_s_ += dt_;
    patrol_yaw_rate_ +=
        std::clamp(PARAM.patrol_yaw_rate_rad_s - patrol_yaw_rate_,
                   -MAX_PATROL_YAW_ACCEL * dt_, MAX_PATROL_YAW_ACCEL * dt_);
    target_world_.Yaw() += patrol_yaw_rate_ * dt_;
    target_yaw_rate_ref_ = patrol_yaw_rate_;
    target_yaw_accel_ref_ = 0.0f;
    target_world_.Pitch() =
        euler_.Pitch() +
        PARAM.patrol_pitch_amplitude_rad *
            std::sin(PARAM.patrol_pitch_angular_rate_rad_s * patrol_elapsed_s_);
  }

  static constexpr float MAX_PATROL_YAW_ACCEL = 0.2f;

  void ControlIdentify() {
    if (system_identify_.GetActiveAxis() == SystemIdentify::ActiveAxis::PITCH) {
      const float margin = system_identify_.GetConfig().pit_angle_margin_rad;
      const float pitch =
          JointPosition(motor_pit_feedback_, PARAM.two_axis_ik.pitch_sign,
                        PARAM.two_axis_ik.pitch_zero_rad);
      if (pitch < PARAM.two_axis_ik.pitch_min_rad + margin ||
          pitch > PARAM.two_axis_ik.pitch_max_rad - margin) {
        system_identify_.Abort(IdentifyAbort::PITCH_ANGLE_LIMIT);
      }
    }
    const SystemIdentify::Sensors sensors{
        .yaw_omega_rad_s =
            PARAM.two_axis_ik.yaw_sign * motor_syaw_feedback_.omega,
        .pit_omega_rad_s =
            PARAM.two_axis_ik.pitch_sign * motor_pit_feedback_.omega,
        .pit_theta_rad = euler_.Pitch(),
        .yaw_motor_online =
            motor_syaw_->IsOnline() && motor_syaw_feedback_.state == 1,
        .pit_motor_online =
            motor_pit_->IsOnline() && motor_pit_feedback_.state == 1,
        .rc_online = cmd_.Online(),
    };
    const float torque = system_identify_.Update(sensors, dt_);
    const auto active = system_identify_.GetActiveAxis();
    const auto command = Motor::MotorCmd(
        {.mode = Motor::ControlMode::MODE_TORQUE,
         .torque = active == SystemIdentify::ActiveAxis::YAW
                       ? PARAM.two_axis_ik.yaw_sign * torque
                       : PARAM.two_axis_ik.pitch_sign * torque});
    if (active == SystemIdentify::ActiveAxis::YAW) {
      motor_pit_->Relax();
      RelaxBigYaw();
      ControlMotor(motor_syaw_, motor_syaw_feedback_, command);
    } else if (active == SystemIdentify::ActiveAxis::PITCH) {
      RelaxBigYaw();
      motor_syaw_->Relax();
      ControlMotor(motor_pit_, motor_pit_feedback_, command);
    } else {
      RelaxBigYaw();
      motor_syaw_->Relax();
      motor_pit_->Relax();
    }
    if (system_identify_.State() == IdentifyState::DONE ||
        system_identify_.State() == IdentifyState::ABORTED) {
      identify_relax_lock_ = true;
      SetMode(GimbalEvent::SET_MODE_RELAX);
    }
  }

  void SetMode(GimbalEvent event) {
    if (identify_relax_lock_ && event != GimbalEvent::SET_MODE_RELAX)
      return;
    if (current_mode_ == GimbalEvent::SET_MODE_IDENTIFY &&
        event == GimbalEvent::SET_MODE_RELAX &&
        system_identify_.State() == IdentifyState::ARMING) {
      return;
    }
    if (current_mode_ == GimbalEvent::SET_MODE_IDENTIFY &&
        event != GimbalEvent::SET_MODE_IDENTIFY &&
        system_identify_.IsActive()) {
      system_identify_.Abort(IdentifyAbort::EXTERNAL_EVENT);
      identify_relax_lock_ = true;
      event = GimbalEvent::SET_MODE_RELAX;
    }
    if (event == current_mode_)
      return;
    current_mode_ = event;
    patrol_elapsed_s_ = 0.0f;
    patrol_yaw_rate_ = 0.0f;
    manual_control_.Reset(last_applied_yaw_torque_nm_,
                          last_applied_syaw_torque_nm_);
    auto_aim_control_.Reset(last_applied_yaw_torque_nm_,
                            last_applied_syaw_torque_nm_);
    SyncTargetToFeedback();
    target_initialized_ = false;
    if (event == GimbalEvent::SET_MODE_RELAX) {
      RelaxAll();
    } else if (event == GimbalEvent::SET_MODE_IDENTIFY) {
      if (PARAM.system_identify.enabled) {
        system_identify_.Begin();
      } else {
        current_mode_ = GimbalEvent::SET_MODE_RELAX;
        RelaxAll();
      }
    }
    uint8_t vision_task = 0U;
    if (event == GimbalEvent::SET_VISION_AUTO_AIM)
      vision_task = 1U;
    if (event == GimbalEvent::SET_VISION_SMALL_BUFF)
      vision_task = 2U;
    if (event == GimbalEvent::SET_VISION_BIG_BUFF)
      vision_task = 3U;
    topic_vision_task_.Publish(vision_task);
  }
};

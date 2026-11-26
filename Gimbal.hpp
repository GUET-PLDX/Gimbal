#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Dual-Yaw gimbal controller with IMU feedback
depends:
- id: GUET-PLDX/CMD
  ref: b0c8eed1a1b601361b5e1c7e63d5a79064cd4760
- id: GUET-PLDX/Motor
- id: xrobot-org/BMI088
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <optional>

#include "CMD.hpp"
#include "Motor.hpp"
#include "SystemIdentify.hpp"
#include "YawSmc.hpp"
#include "event.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "mutex.hpp"
#include "latest_snapshot.hpp"
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
  enum class AutoAimYawState : uint8_t { INACTIVE, HOLD, RECENTER };

  struct AutoAimYawStatus {
    AutoAimYawState state = AutoAimYawState::INACTIVE;
    // Raw big Yaw encoder angle, rad; valid while HOLD or RECENTER.
    float hold_encoder_angle_rad = 0.0f;
  };

  struct ControlParam {
    LibXR::PID<float>::Param pid_pit_angle;
    LibXR::PID<float>::Param pid_pit_omega;
    YawSmc::Param yaw_smc;
    YawSmc::Param syaw_smc;
    YawSmc::Param fold_smc;
  };

  struct PatrolParam {
    float big_yaw_period_s;
    float small_yaw_amplitude_rad;
    float small_yaw_period_s;
    float pitch_amplitude_rad;
    float pitch_period_s;
    const char* big_yaw_euler_topic_name;
    const char* big_yaw_gyro_topic_name;
  };

  struct Param {
    uint32_t task_stack_depth;
    ControlParam manual;
    ControlParam auto_aim;
    float yaw_torque_limit_nm;
    std::optional<float> syaw_torque_limit_nm;
    float fold_torque_limit_nm;
    float yaw_zero;
    float syaw_zero;
    float pit_zero;
    float pitch_min_rad;
    float pitch_max_rad;
    float small_yaw_min_rad;
    float small_yaw_max_rad;
    float fold_bottom_rad;
    float fold_top_rad;
    float fold_tolerance_rad;
    float yaw_follow_deadband_rad;
    float auto_aim_recenter_trigger_min_rad;
    float auto_aim_recenter_trigger_max_rad;
    float fold_ramp_rad_s;
    float pit_lc;
    float pit_theta;
    float j_pit;
    float j_yaw;
    float j_syaw;
    float j_fold;
    float patrol_pitch_amplitude_rad;
    float patrol_pitch_angular_rate_rad_s;
    float patrol_yaw_rate_rad_s;
    LibXR::Thread::Priority thread_priority;
    SystemIdentify::Config system_identify;
    const char* euler_topic_name;
    const char* gyro_topic_name;
    const char* gimbal_cmd_topic_name;
    PatrolParam patrol;
    bool manual_configuration_interlock = false;
  };

  Gimbal(CMD& cmd, Motor& motor_pit, Motor& motor_yaw, Motor& motor_syaw,
         Motor& motor_fold,
         const Param& param =
             Param{.task_stack_depth = 2048,
                   .manual = {},
                   .auto_aim = {},
                   .yaw_torque_limit_nm = 2.0f,
                   .syaw_torque_limit_nm = {},
                   .fold_torque_limit_nm = 0.0f,
                   .yaw_zero = 0.0f,
                   .syaw_zero = 0.0f,
                   .pit_zero = 0.0f,
                   .pitch_min_rad = -static_cast<float>(LibXR::PI / 2.0),
                   .pitch_max_rad = static_cast<float>(LibXR::PI / 2.0),
                   .small_yaw_min_rad = 0.0f,
                   .small_yaw_max_rad = 0.0f,
                   .fold_bottom_rad = 0.0f,
                   .fold_top_rad = 0.0f,
                   .fold_tolerance_rad = 0.0f,
                   .yaw_follow_deadband_rad = 0.0f,
                   .auto_aim_recenter_trigger_min_rad = 0.0f,
                   .auto_aim_recenter_trigger_max_rad = 0.0f,
                   .fold_ramp_rad_s = 0.0f,
                   .pit_lc = 0.0f,
                   .pit_theta = 0.0f,
                   .j_pit = 0.0f,
                   .j_yaw = 0.0f,
                   .j_syaw = 0.0f,
                   .j_fold = 0.0f,
                   .patrol_pitch_amplitude_rad = 0.0f,
                   .patrol_pitch_angular_rate_rad_s = 0.0f,
                   .patrol_yaw_rate_rad_s = 0.0f,
                   .thread_priority = LibXR::Thread::Priority::MEDIUM,
                   .system_identify = {},
                   .euler_topic_name = "gimbal_euler",
                   .gyro_topic_name = "gimbal_gyro",
                   .gimbal_cmd_topic_name = "gimbal_cmd",
                   .patrol = {},
                   .manual_configuration_interlock = false})
      : cmd_(cmd),
        PARAM(param),
        manual_control_(PARAM.manual, PARAM),
        auto_aim_control_(PARAM.auto_aim, PARAM),
        fold_smc_(PARAM.manual.fold_smc, PARAM.fold_torque_limit_nm, false),
        motor_yaw_(&motor_yaw),
        motor_pit_(&motor_pit),
        motor_syaw_(&motor_syaw),
        motor_fold_(&motor_fold),
        system_identify_(param.system_identify) {
    ASSERT(PARAM.small_yaw_min_rad <= PARAM.auto_aim_recenter_trigger_min_rad &&
           PARAM.auto_aim_recenter_trigger_min_rad <= 0.0f &&
           PARAM.auto_aim_recenter_trigger_max_rad >= 0.0f &&
           PARAM.auto_aim_recenter_trigger_max_rad <= PARAM.small_yaw_max_rad);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool, Gimbal* gimbal, uint32_t) {
          LibXR::Mutex::LockGuard guard(gimbal->control_mutex_);
          gimbal->SetMode(GimbalEvent::SET_MODE_RELAX);
        },
        this);
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Gimbal* gimbal, uint32_t event_id) {
          ASSERT(!in_isr);
          if (in_isr) return;
          LibXR::Mutex::LockGuard guard(gimbal->control_mutex_);
          gimbal->SetMode(static_cast<GimbalEvent>(event_id));
        },
        this);
    auto cmd_callback = LibXR::Topic::Callback::Create(
        [](bool, Gimbal* gimbal, const CMD::GimbalCMD& data) {
          gimbal->latest_cmd_.Store(data);
        }, this);
    LibXR::Topic(LibXR::Topic::Find(PARAM.gimbal_cmd_topic_name))
        .RegisterCallback(cmd_callback);
    cmd_.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    for (uint32_t event = 0;
         event <= static_cast<uint32_t>(GimbalEvent::SET_MODE_IDENTIFY);
         ++event) {
      gimbal_event_.Register(event, callback);
    }
    thread_.Create(this, ThreadFunc, "GimbalThread", param.task_stack_depth,
                   PARAM.thread_priority);
  }

  static void ThreadFunc(Gimbal* gimbal) {
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        gimbal->PARAM.euler_topic_name);
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        gimbal->PARAM.gyro_topic_name);
    LibXR::Topic::ASyncSubscriber<Pldx::LifterPosition> fold_target_suber(
        LibXR::Topic(LibXR::Topic::FindOrCreate<Pldx::LifterPosition>(
            Pldx::LIFTER_TARGET_TOPIC)));
    std::optional<LibXR::Topic::ASyncSubscriber<Pldx::ConfigurationFoldCommand>>
        configuration_fold_suber;
    if (gimbal->PARAM.manual_configuration_interlock) {
      configuration_fold_suber.emplace(LibXR::Topic(
          LibXR::Topic::FindOrCreate<Pldx::ConfigurationFoldCommand>(
              Pldx::CONFIGURATION_FOLD_COMMAND_TOPIC)));
      configuration_fold_suber->StartWaiting();
    }
    std::optional<LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>>>
        big_yaw_euler_suber;
    std::optional<LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>>>
        big_yaw_gyro_suber;
    if (gimbal->PARAM.patrol.big_yaw_euler_topic_name &&
        gimbal->PARAM.patrol.big_yaw_gyro_topic_name) {
      big_yaw_euler_suber.emplace(
          gimbal->PARAM.patrol.big_yaw_euler_topic_name);
      big_yaw_gyro_suber.emplace(gimbal->PARAM.patrol.big_yaw_gyro_topic_name);
      big_yaw_euler_suber->StartWaiting();
      big_yaw_gyro_suber->StartWaiting();
    }

    euler_suber.StartWaiting();
    gyro_suber.StartWaiting();
    fold_target_suber.StartWaiting();

    gimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    auto last_time = LibXR::Timebase::GetMilliseconds();
    uint8_t last_mode;
    {
      LibXR::Mutex::LockGuard guard(gimbal->control_mutex_);
      last_mode = static_cast<uint8_t>(gimbal->current_mode_);
      gimbal->topic_mode_.Publish(last_mode);
    }

    while (true) {
      {
        LibXR::Mutex::LockGuard guard(gimbal->control_mutex_);
        gimbal->latest_cmd_.LoadLatest(gimbal->cmd_data_);
        if (euler_suber.Available()) {
          gimbal->euler_ = euler_suber.GetData();
          gimbal->euler_.Pitch() *= -1.0f;
          gimbal->euler_received_ = true;
          euler_suber.StartWaiting();
        }
        if (gyro_suber.Available()) {
          gimbal->gyro_data_ = gyro_suber.GetData();
          gimbal->gyro_data_.y() *= -1.0f;
          gimbal->gyro_received_ = true;
          gyro_suber.StartWaiting();
        }
        if (fold_target_suber.Available()) {
          gimbal->fold_target_ = fold_target_suber.GetData();
          fold_target_suber.StartWaiting();
        }
        if (configuration_fold_suber && configuration_fold_suber->Available()) {
          gimbal->configuration_fold_ = configuration_fold_suber->GetData();
          configuration_fold_suber->StartWaiting();
        }
        if (big_yaw_euler_suber && big_yaw_euler_suber->Available()) {
          gimbal->big_yaw_euler_ = big_yaw_euler_suber->GetData();
          gimbal->big_yaw_euler_received_ = true;
          big_yaw_euler_suber->StartWaiting();
        }
        if (big_yaw_gyro_suber && big_yaw_gyro_suber->Available()) {
          gimbal->big_yaw_gyro_ = big_yaw_gyro_suber->GetData();
          gimbal->big_yaw_gyro_received_ = true;
          big_yaw_gyro_suber->StartWaiting();
        }
        gimbal->Update();
        gimbal->ParseCMD();
        gimbal->Control();
        uint8_t mode = static_cast<uint8_t>(gimbal->current_mode_);
        if (mode != last_mode) {
          gimbal->topic_mode_.Publish(mode);
          last_mode = mode;
        }
      }
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
    const bool yaw_online = motor_yaw_->Update() == LibXR::ErrorCode::OK;
    motor_pit_->Update();
    motor_syaw_->Update();
    fold_motor_online_ = motor_fold_->Update() == LibXR::ErrorCode::OK;
    motor_yaw_feedback_ = motor_yaw_->GetFeedback();
    motor_pit_feedback_ = motor_pit_->GetFeedback();
    motor_syaw_feedback_ = motor_syaw_->GetFeedback();
    fold_motor_feedback_ = motor_fold_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;

    float big_yaw = motor_yaw_feedback_.abs_angle - PARAM.yaw_zero;
    pitch_relative_angle_ = motor_pit_feedback_.abs_angle - PARAM.pit_zero;
    small_yaw_relative_angle_ =
        motor_syaw_feedback_.abs_angle - PARAM.syaw_zero;

    Pldx::LifterFeedback lifter_feedback;
    if (PARAM.fold_top_rad > PARAM.fold_bottom_rad && fold_motor_online_ &&
        fold_motor_feedback_.received_time_us != 0U) {
      const float angle = fold_motor_feedback_.position;
      if (std::fabs(angle - PARAM.fold_top_rad) <= PARAM.fold_tolerance_rad) {
        lifter_feedback.position = Pldx::LifterPosition::TOP;
      } else if (std::fabs(angle - PARAM.fold_bottom_rad) <=
                 PARAM.fold_tolerance_rad) {
        lifter_feedback.position = Pldx::LifterPosition::BOTTOM;
        lifter_feedback.transform_percent = 100.0f;
      } else {
        lifter_feedback.transform_percent =
            100.0f *
            std::clamp(1.0f - std::fabs(angle - PARAM.fold_bottom_rad) /
                                  (PARAM.fold_top_rad - PARAM.fold_bottom_rad),
                       0.0f, 1.0f);
      }
      lifter_feedback.is_transformable =
          fold_motor_feedback_.error_id == 0U &&
          lifter_feedback.position != Pldx::LifterPosition::MIDDLE;
    }
    fold_feedback_topic_.Publish(lifter_feedback);
    if (PARAM.manual_configuration_interlock) {
      Pldx::ConfigurationFeedback feedback{
          .position = lifter_feedback.position,
          .yaw_rad = big_yaw,
          .fold_valid = FoldControlAvailable() && fold_motor_online_ &&
                        fold_motor_feedback_.state == 1U &&
                        fold_motor_feedback_.error_id == 0U &&
                        std::isfinite(fold_motor_feedback_.position) &&
                        std::isfinite(fold_motor_feedback_.omega),
          .yaw_valid = yaw_online && motor_yaw_feedback_.error_id == 0U &&
                       std::isfinite(big_yaw),
          .fold_sample_us = fold_motor_feedback_.received_time_us,
          .yaw_sample_us = motor_yaw_feedback_.received_time_us};
      configuration_feedback_topic_.Publish(feedback);
    }
    topic_yaw_angle_.Publish(big_yaw);
    topic_pit_angle_.Publish(pitch_relative_angle_);
    topic_syaw_angle_.Publish(small_yaw_relative_angle_);
    topic_yaw_omega_.Publish(motor_yaw_feedback_.omega);
    topic_pit_omega_.Publish(motor_pit_feedback_.omega);
    topic_syaw_omega_.Publish(motor_syaw_feedback_.omega);
  }

  void ParseCMD() {
    const auto ctrl_mode = cmd_data_.control_mode;
    /* COMMON 使用摇杆积分；旧视觉角尚未被新手动帧替换时先撤销输出。
       AUTO 模式下的手动快照仍可沿用既有的操作员接管／视觉掉线回退。 */
    waiting_remote_frame_ = current_mode_ == GimbalEvent::SET_MODE_COMMON &&
                            ctrl_mode == CMD::Mode::CMD_AUTO_CTRL &&
                            cmd_data_.ai_online;
    if (waiting_remote_frame_) {
      target_initialized_ = false;
      return;
    }
    const bool operator_control = ctrl_mode == CMD::Mode::CMD_OP_CTRL;
    if (operator_control && current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
      SetMode(GimbalEvent::SET_MODE_COMMON);
    }
    if (current_mode_ != GimbalEvent::SET_MODE_RELAX && !target_initialized_) {
      if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL &&
          (!euler_received_ || !gyro_received_ || !big_yaw_euler_received_ ||
           !big_yaw_gyro_received_)) {
        return;
      }
      target_world_yaw_ = euler_.Yaw();
      target_world_pitch_ = euler_.Pitch();
      if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
        const float big_rate =
            static_cast<float>(LibXR::TWO_PI) / PARAM.patrol.big_yaw_period_s;
        const float joint_rate = 4.0f * PARAM.patrol.small_yaw_amplitude_rad /
                                 PARAM.patrol.small_yaw_period_s;
        patrol_sweep_rate_ =
            (joint_rate + std::hypot(joint_rate, 2.0f * big_rate)) / 2.0f;
        patrol_forward_time_s_ = PARAM.patrol.small_yaw_period_s *
                                 (1.0f + big_rate / patrol_sweep_rate_) / 2.0f;
        patrol_relative_ref_ = small_yaw_relative_angle_;
        patrol_yaw_phase_s_ =
            (patrol_relative_ref_ + PARAM.patrol.small_yaw_amplitude_rad) /
            (patrol_sweep_rate_ - big_rate);
        target_big_world_yaw_ = big_yaw_euler_.Yaw();
        target_world_pitch_ = 0.0f;
        target_yaw_rate_ref_ = patrol_yaw_phase_s_ < patrol_forward_time_s_
                                   ? patrol_sweep_rate_
                                   : -patrol_sweep_rate_;
        target_yaw_accel_ref_ = 0.0f;
      }
      target_initialized_ = true;
      return;
    }
    const bool ai_active =
        ctrl_mode == CMD::Mode::CMD_AUTO_CTRL && cmd_data_.ai_online &&
        (current_mode_ == GimbalEvent::SET_VISION_AUTO_AIM ||
         current_mode_ == GimbalEvent::SET_VISION_SMALL_BUFF ||
         current_mode_ == GimbalEvent::SET_VISION_BIG_BUFF);
    if (ai_active != ai_gimbal_reference_active_) {
      ai_gimbal_reference_active_ = ai_active;
      auto& control = ai_active ? auto_aim_control_ : manual_control_;
      control.Reset();
      target_world_yaw_ = euler_.Yaw();
      target_world_pitch_ = euler_.Pitch();
      target_yaw_rate_ref_ = 0.0f;
      target_yaw_accel_ref_ = 0.0f;
      return;
    }
    if (ai_active) {
      target_world_yaw_ = cmd_data_.yaw;
      target_world_pitch_ = cmd_data_.pit;
      target_yaw_rate_ref_ = cmd_data_.yaw_dot;
      target_yaw_accel_ref_ = cmd_data_.yaw_ddot;
      return;
    }
    if (!operator_control &&
        current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
      if (!std::isfinite(dt_) || dt_ <= 0.0f) {
        return;
      }
      const float big_rate =
          static_cast<float>(LibXR::TWO_PI) / PARAM.patrol.big_yaw_period_s;
      patrol_yaw_phase_s_ =
          std::fmod(patrol_yaw_phase_s_ + dt_, PARAM.patrol.small_yaw_period_s);
      const bool forward = patrol_yaw_phase_s_ < patrol_forward_time_s_;
      const float relative_ref =
          forward ? -PARAM.patrol.small_yaw_amplitude_rad +
                        2.0f * PARAM.patrol.small_yaw_amplitude_rad *
                            patrol_yaw_phase_s_ / patrol_forward_time_s_
                  : PARAM.patrol.small_yaw_amplitude_rad -
                        2.0f * PARAM.patrol.small_yaw_amplitude_rad *
                            (patrol_yaw_phase_s_ - patrol_forward_time_s_) /
                            (PARAM.patrol.small_yaw_period_s -
                             patrol_forward_time_s_);
      // Integrate both world references from their own IMU origins. The
      // relative increment keeps the encoder-defined sweep independent of their
      // yaw zeros.
      target_big_world_yaw_ =
          LibXR::CycleValue<float>(target_big_world_yaw_ + big_rate * dt_);
      target_world_yaw_ =
          LibXR::CycleValue<float>(target_world_yaw_ + big_rate * dt_ +
                                   relative_ref - patrol_relative_ref_);
      patrol_relative_ref_ = relative_ref;
      target_yaw_rate_ref_ = forward ? patrol_sweep_rate_ : -patrol_sweep_rate_;
      // Ideal velocity steps have no finite acceleration impulse feedforward.
      target_yaw_accel_ref_ = 0.0f;
      patrol_elapsed_s_ =
          std::fmod(patrol_elapsed_s_ + dt_, PARAM.patrol.pitch_period_s);
      target_world_pitch_ =
          PARAM.patrol.pitch_amplitude_rad *
          std::sin(static_cast<float>(LibXR::TWO_PI) * patrol_elapsed_s_ /
                   PARAM.patrol.pitch_period_s);
      return;
    }
    const float yaw_delta = cmd_data_.yaw * dt_ * GIMBAL_MAX_SPEED;
    target_world_yaw_ += operator_control ? yaw_delta : -yaw_delta;
    target_world_pitch_ += cmd_data_.pit * dt_ * GIMBAL_MAX_SPEED;
    target_yaw_rate_ref_ = 0.0f;
    target_yaw_accel_ref_ = 0.0f;
  }

  void Control() {
    if (waiting_remote_frame_) {
      motor_yaw_->Relax();
      motor_pit_->Relax();
      motor_syaw_->Relax();
      motor_fold_->Relax();
      big_yaw_relaxed_ = true;
      fold_engaged_ = false;
      fold_reference_rad_.reset();
      fold_hold_rad_.reset();
      manual_control_.Reset();
      auto_aim_control_.Reset();
      auto_aim_yaw_ = {};
      topic_auto_aim_yaw_.Publish(auto_aim_yaw_);
      return;
    }
    const float big_yaw_angle = motor_yaw_feedback_.abs_angle;
    const bool ordinary_auto_aim =
        current_mode_ == GimbalEvent::SET_VISION_AUTO_AIM &&
        cmd_data_.control_mode == CMD::Mode::CMD_AUTO_CTRL &&
        cmd_data_.ai_online;
    if (!ordinary_auto_aim) {
      auto_aim_yaw_ = {};
    } else {
      if (auto_aim_yaw_.state == AutoAimYawState::INACTIVE &&
          std::isfinite(big_yaw_angle)) {
        auto_aim_yaw_.state = AutoAimYawState::HOLD;
        auto_aim_yaw_.hold_encoder_angle_rad = big_yaw_angle;
      }
      if (auto_aim_yaw_.state == AutoAimYawState::HOLD &&
          (small_yaw_relative_angle_ <
               PARAM.auto_aim_recenter_trigger_min_rad ||
           small_yaw_relative_angle_ >
               PARAM.auto_aim_recenter_trigger_max_rad)) {
        auto_aim_yaw_.state = AutoAimYawState::RECENTER;
      } else if (auto_aim_yaw_.state == AutoAimYawState::RECENTER &&
                 std::fabs(small_yaw_relative_angle_) <=
                     PARAM.yaw_follow_deadband_rad &&
                 std::isfinite(big_yaw_angle)) {
        auto_aim_yaw_.state = AutoAimYawState::HOLD;
        auto_aim_yaw_.hold_encoder_angle_rad = big_yaw_angle;
      }
    }
    topic_auto_aim_yaw_.Publish(auto_aim_yaw_);

    const auto motor_control = [this](Motor* motor,
                                      const Motor::Feedback& feedback,
                                      const Motor::MotorCmd& command) {
      if (feedback.state == 0) {
        motor->Enable();
      } else if (feedback.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(command);
      }
      if (motor == motor_yaw_) {
        big_yaw_relaxed_ = false;
      }
    };

    const auto selection = cmd_.GetControlSelection();
    const bool manual_fold_control = PARAM.manual_configuration_interlock &&
                                     selection.mode == CMD::Mode::CMD_OP_CTRL;
    if (PARAM.manual_configuration_interlock &&
        selection.revision != last_fold_control_revision_) {
      last_fold_control_revision_ = selection.revision;
      fold_reference_rad_.reset();
      fold_hold_rad_.reset();
    }
    const auto fold_action =
        PARAM.manual_configuration_interlock && cmd_.ChassisRelaxRequested()
            ? Pldx::FoldAction::HOLD
            : SelectFoldAction(manual_fold_control, selection.revision);
    if (fold_action != last_fold_action_) {
      if (PARAM.manual_configuration_interlock) {
        fold_reference_rad_.reset();
        fold_hold_rad_.reset();
      }
      last_fold_action_ = fold_action;
    }
    const bool fold_feedback_usable =
        fold_motor_online_ && std::isfinite(fold_motor_feedback_.position) &&
        std::isfinite(fold_motor_feedback_.omega);
    if (fold_action != Pldx::FoldAction::RELEASE && FoldControlAvailable() &&
        (!manual_fold_control || fold_feedback_usable)) {
      if (!fold_motor_online_ ||
          !std::isfinite(fold_motor_feedback_.position)) {
        fold_reference_rad_.reset();
        fold_hold_rad_.reset();
      } else if (fold_motor_feedback_.state != 1U) {
        fold_reference_rad_.reset();
        fold_hold_rad_.reset();
        motor_control(motor_fold_, fold_motor_feedback_, {});
        fold_engaged_ = true;
      } else if (fold_motor_feedback_.received_time_us != 0U) {
        const float angle = fold_motor_feedback_.position;
        if (fold_action == Pldx::FoldAction::HOLD && !fold_hold_rad_) {
          fold_hold_rad_ = angle;
        }
        float target = PARAM.fold_bottom_rad;
        if (fold_hold_rad_) {
          target = *fold_hold_rad_;
        } else if (fold_action == Pldx::FoldAction::TOP) {
          target = PARAM.fold_top_rad;
        }
        const float reference = fold_reference_rad_.value_or(angle);
        const float max_step = PARAM.fold_ramp_rad_s * dt_;
        const float step = std::clamp(target - reference, -max_step, max_step);
        fold_reference_rad_ = reference + step;
        const float torque = fold_smc_.Calculate(
            {.theta_rad = *fold_reference_rad_,
             .omega_rad_s = dt_ > 0.0f ? step / dt_ : 0.0f},
            {.theta_rad = angle, .omega_rad_s = fold_motor_feedback_.omega},
            PARAM.j_fold);
        motor_control(
            motor_fold_, fold_motor_feedback_,
            {.mode = Motor::ControlMode::MODE_TORQUE, .torque = torque});
        fold_engaged_ = true;
      }
    } else if (fold_engaged_) {
      motor_fold_->Relax();
      fold_reference_rad_.reset();
      fold_hold_rad_.reset();
      fold_engaged_ = false;
    }

    switch (current_mode_) {
      case GimbalEvent::SET_MODE_RELAX:
        break;
      case GimbalEvent::SET_MODE_IDENTIFY:
        if (PARAM.system_identify.enabled) {
          ControlIdentify(motor_control);
          return;
        }
        break;
      default: {
        const bool patrol = current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL;
        if (patrol && !target_initialized_) {
          break;
        }
        float target_pit_angle = target_world_pitch_;
        PitchLimit(target_pit_angle, euler_.Pitch(), pitch_relative_angle_,
                   PARAM.pitch_max_rad, PARAM.pitch_min_rad, 1.0f);
        float pit_output = 0.0f;
        float yaw_output = 0.0f;
        Solve(pit_output, yaw_output, target_pit_angle, target_world_yaw_, dt_);

        auto& control =
            ai_gimbal_reference_active_ ? auto_aim_control_ : manual_control_;
        const float syaw_follow_error =
            std::fabs(small_yaw_relative_angle_) <=
                    PARAM.yaw_follow_deadband_rad
                ? 0.0f
                : small_yaw_relative_angle_ -
                      std::copysign(PARAM.yaw_follow_deadband_rad,
                                    small_yaw_relative_angle_);
        const bool hold = auto_aim_yaw_.state == AutoAimYawState::HOLD;
        const float big_yaw_rate =
            patrol ? (big_yaw_gyro_.y() * std::sin(big_yaw_euler_.Roll()) +
                      big_yaw_gyro_.z() * std::cos(big_yaw_euler_.Roll())) /
                         std::cos(big_yaw_euler_.Pitch())
                   : motor_yaw_feedback_.omega;
        const float big_yaw_output = control.yaw_smc.Calculate(
            {.theta_rad = patrol ? target_big_world_yaw_
                          : hold ? auto_aim_yaw_.hold_encoder_angle_rad
                                 : big_yaw_angle + syaw_follow_error,
             .omega_rad_s = patrol ? static_cast<float>(LibXR::TWO_PI) /
                                         PARAM.patrol.big_yaw_period_s
                            : hold ? 0.0f
                                   : motor_yaw_feedback_.omega +
                                         (syaw_follow_error == 0.0f
                                              ? 0.0f
                                              : motor_syaw_feedback_.omega),
             .alpha_rad_s2 = 0.0f},
            {.theta_rad = patrol ? big_yaw_euler_.Yaw() : big_yaw_angle,
             .omega_rad_s = big_yaw_rate},
            PARAM.j_yaw);

        motor_control(motor_yaw_, motor_yaw_feedback_,
                      {.mode = Motor::ControlMode::MODE_TORQUE,
                       .torque = big_yaw_output});
        motor_control(
            motor_syaw_, motor_syaw_feedback_,
            {.mode = Motor::ControlMode::MODE_TORQUE, .torque = yaw_output});
        motor_control(
            motor_pit_, motor_pit_feedback_,
            {.mode = Motor::ControlMode::MODE_TORQUE,
             .torque = std::isfinite(pit_output) ? pit_output : 0.0f});
        return;
      }
    }

    if (!big_yaw_relaxed_) {
      motor_yaw_->Relax();
      big_yaw_relaxed_ = true;
    }

    motor_pit_->Relax();
    motor_syaw_->Relax();
    manual_control_.Reset();
    auto_aim_control_.Reset();
  }

  LibXR::Event& GetEvent() { return gimbal_event_; }

 private:
  bool FoldControlAvailable() const {
    return PARAM.fold_torque_limit_nm > 0.0F &&
           PARAM.fold_top_rad > PARAM.fold_bottom_rad &&
           current_mode_ != GimbalEvent::SET_MODE_RELAX &&
           current_mode_ != GimbalEvent::SET_MODE_IDENTIFY;
  }

  Pldx::FoldAction SelectFoldAction(bool manual_fold_control,
                                    uint32_t control_revision) const {
    if (manual_fold_control) {
      const auto now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      if (!configuration_fold_ ||
          configuration_fold_->control_revision != control_revision ||
          now_ms - configuration_fold_->stamp_ms >
              Pldx::CONFIGURATION_COMMAND_TIMEOUT_MS) {
        return Pldx::FoldAction::HOLD;
      }
      return configuration_fold_->action;
    }
    if (!fold_target_) return Pldx::FoldAction::RELEASE;
    return *fold_target_ == Pldx::LifterPosition::TOP
               ? Pldx::FoldAction::TOP
               : Pldx::FoldAction::BOTTOM;
  }

  struct ControlState {
    LibXR::PID<float> pid_pit_angle;
    LibXR::PID<float> pid_pit_omega;
    YawSmc yaw_smc;
    YawSmc syaw_smc;
    float last_pitch_position_speed = 0.0f;
    bool pitch_reference_primed = false;

    ControlState(const ControlParam& param, const Param& limits)
        : pid_pit_angle(param.pid_pit_angle),
          pid_pit_omega(param.pid_pit_omega),
          yaw_smc(param.yaw_smc, limits.yaw_torque_limit_nm),
          syaw_smc(param.syaw_smc, limits.syaw_torque_limit_nm.value_or(
                                       limits.yaw_torque_limit_nm)) {}

    void Reset() {
      pid_pit_angle.Reset();
      pid_pit_omega.Reset();
      pid_pit_omega.SetFeedForward(0.0f);
      last_pitch_position_speed = 0.0f;
      pitch_reference_primed = false;
    }
  };

  CMD::GimbalCMD cmd_data_{};
  LibXR::MicrosecondTimestamp last_online_time_{};
  float dt_ = 0.0f;

  float target_world_yaw_ = 0.0f;
  float target_big_world_yaw_ = 0.0f;
  float target_world_pitch_ = 0.0f;
  float target_yaw_rate_ref_ = 0.0f;
  float target_yaw_accel_ref_ = 0.0f;
  bool target_initialized_ = false;

  LibXR::EulerAngle<float> euler_;
  Eigen::Matrix<float, 3, 1> gyro_data_ = Eigen::Matrix<float, 3, 1>::Zero();
  bool euler_received_ = false;
  bool gyro_received_ = false;
  LibXR::EulerAngle<float> big_yaw_euler_;
  Eigen::Matrix<float, 3, 1> big_yaw_gyro_ = Eigen::Matrix<float, 3, 1>::Zero();
  bool big_yaw_euler_received_ = false;
  bool big_yaw_gyro_received_ = false;

  float small_yaw_relative_angle_ = 0.0f;
  float pitch_relative_angle_ = 0.0f;
  Motor::Feedback motor_yaw_feedback_{};
  Motor::Feedback motor_syaw_feedback_{};
  Motor::Feedback motor_pit_feedback_{};

  bool big_yaw_relaxed_ = false;

  bool ai_gimbal_reference_active_ = false;
  AutoAimYawStatus auto_aim_yaw_;

  float patrol_elapsed_s_ = 0.0f;
  float patrol_yaw_phase_s_ = 0.0f;
  float patrol_relative_ref_ = 0.0f;
  float patrol_sweep_rate_ = 0.0f;
  float patrol_forward_time_s_ = 0.0f;

  bool identify_boot_checked_ = false;
  bool identify_relax_lock_ = false;
  bool waiting_remote_frame_ = false; /* SC 事件与命令尚未一致时撤销输出 */

  std::optional<Pldx::LifterPosition> fold_target_;
  std::optional<Pldx::ConfigurationFoldCommand> configuration_fold_;
  uint32_t last_fold_control_revision_ = 0U;
  Motor::Feedback fold_motor_feedback_{};
  bool fold_motor_online_ = false;
  std::optional<float> fold_reference_rad_;
  std::optional<float> fold_hold_rad_;
  Pldx::FoldAction last_fold_action_ = Pldx::FoldAction::RELEASE;
  bool fold_engaged_ = false;

  CMD& cmd_;
  const Param PARAM;
  ControlState manual_control_;
  ControlState auto_aim_control_;
  YawSmc fold_smc_;
  LibXR::Topic fold_feedback_topic_{
      LibXR::Topic::FindOrCreate<Pldx::LifterFeedback>(
          Pldx::LIFTER_FEEDBACK_TOPIC)};
  LibXR::Topic configuration_feedback_topic_{
      LibXR::Topic::FindOrCreate<Pldx::ConfigurationFeedback>(
          Pldx::CONFIGURATION_FEEDBACK_TOPIC)};

  Motor* motor_yaw_;
  Motor* motor_pit_;
  Motor* motor_syaw_;
  Motor* motor_fold_;
  SystemIdentify system_identify_;

  static constexpr float GIMBAL_MAX_SPEED =
      static_cast<float>(LibXR::TWO_PI) * 2.0f;

  LibXR::Event gimbal_event_;
  GimbalEvent current_mode_ = GimbalEvent::SET_MODE_RELAX;
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
  LibXR::Topic topic_auto_aim_yaw_ =
      LibXR::Topic::CreateTopic<AutoAimYawStatus>("gimbal_auto_aim_yaw",
                                                  nullptr, true);
  LibXR::Mutex control_mutex_; /* 模式进入动作与控制计算互斥 */
  LibXR::LatestSnapshot<CMD::GimbalCMD> latest_cmd_{CMD::GimbalCMD{}};
  LibXR::Thread thread_;

  void PitchLimit(float& target_pit, float now_eulr_angle,
                  float now_motor_angle, float motor_max, float motor_min,
                  float sign) {
    if (motor_max == 0.0f && motor_min == 0.0f) {
      return;
    }
    const LibXR::CycleValue<float> cycle_motor_min(motor_min);
    const LibXR::CycleValue<float> cycle_motor_max(motor_max);
    const float pitch_bound_0 =
        now_eulr_angle + (cycle_motor_min - now_motor_angle) / sign;
    const float pitch_bound_1 =
        now_eulr_angle + (cycle_motor_max - now_motor_angle) / sign;
    target_pit = std::clamp(target_pit, std::min(pitch_bound_0, pitch_bound_1),
                            std::max(pitch_bound_0, pitch_bound_1));
  }

  void Solve(float& pit_output, float& yaw_output, float target_pit_angle,
             const LibXR::CycleValue<float>& target_yaw_angle, float dt) {
    auto& control =
        ai_gimbal_reference_active_ ? auto_aim_control_ : manual_control_;
    const float measured_pitch_rate = gyro_data_.y();
    const bool pitch_visual_feedforward_active =
        ai_gimbal_reference_active_ && target_pit_angle == target_world_pitch_;
    const float pitch_rate_feedforward =
        pitch_visual_feedforward_active ? cmd_data_.pit_dot : 0.0f;
    const float pitch_position_speed = control.pid_pit_angle.Calculate(
        target_pit_angle, euler_.Pitch(), measured_pitch_rate, dt);
    const float pitch_speed_ref = pitch_position_speed + pitch_rate_feedforward;
    const float pitch_position_accel =
        control.pitch_reference_primed
            ? (pitch_position_speed - control.last_pitch_position_speed) / dt
            : 0.0f;
    const float pitch_accel_ref =
        pitch_position_accel +
        (pitch_visual_feedforward_active ? cmd_data_.pit_ddot : 0.0f);
    control.pid_pit_omega.SetFeedForward(
        PARAM.j_pit * pitch_accel_ref -
        PARAM.pit_lc * std::sin(euler_.Pitch() + PARAM.pit_theta));
    pit_output = control.pid_pit_omega.Calculate(pitch_speed_ref,
                                                 measured_pitch_rate, dt);

    yaw_output = control.syaw_smc.Calculate(
        {.theta_rad = target_yaw_angle,
         .omega_rad_s = target_yaw_rate_ref_,
         .alpha_rad_s2 = target_yaw_accel_ref_},
        {.theta_rad = euler_.Yaw(),
         // ThreadFunc reverses application Pitch and gyro.y. Undo the latter
         // for the ZYX yaw derivative; gyro.z alone loses yaw rate when
         // pitched.
         .omega_rad_s = current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL
                            ? (-gyro_data_.y() * std::sin(euler_.Roll()) +
                               gyro_data_.z() * std::cos(euler_.Roll())) /
                                  std::cos(euler_.Pitch())
                            : gyro_data_.z()},
        PARAM.j_syaw);

    control.last_pitch_position_speed = pitch_position_speed;
    control.pitch_reference_primed = true;
  }

  void ControlIdentify(const auto& motor_control) {
    if (system_identify_.GetActiveAxis() == SystemIdentify::ActiveAxis::PITCH) {
      const float margin = system_identify_.GetConfig().pit_angle_margin_rad;
      if (pitch_relative_angle_ < PARAM.pitch_min_rad + margin ||
          pitch_relative_angle_ > PARAM.pitch_max_rad - margin) {
        system_identify_.Abort(IdentifyAbort::PITCH_ANGLE_LIMIT);
      }
    }
    const SystemIdentify::Sensors sensors{
        .yaw_omega_rad_s = motor_syaw_feedback_.omega,
        .pit_omega_rad_s = motor_pit_feedback_.omega,
        .pit_theta_rad = euler_.Pitch(),
        .yaw_motor_online =
            motor_syaw_->IsOnline() && motor_syaw_feedback_.state == 1,
        .pit_motor_online =
            motor_pit_->IsOnline() && motor_pit_feedback_.state == 1,
        .rc_online = cmd_.Online(),
    };
    const Motor::MotorCmd command{
        .mode = Motor::ControlMode::MODE_TORQUE,
        .torque = system_identify_.Update(sensors, dt_)};
    const auto active_axis = system_identify_.GetActiveAxis();
    if (active_axis == SystemIdentify::ActiveAxis::YAW) {
      motor_pit_->Relax();
    }

    if (!big_yaw_relaxed_) {
      motor_yaw_->Relax();
      big_yaw_relaxed_ = true;
    }

    if (active_axis == SystemIdentify::ActiveAxis::YAW) {
      motor_control(motor_syaw_, motor_syaw_feedback_, command);
    } else {
      motor_syaw_->Relax();
      if (active_axis == SystemIdentify::ActiveAxis::PITCH) {
        motor_control(motor_pit_, motor_pit_feedback_, command);
      } else {
        motor_pit_->Relax();
      }
    }
    if (system_identify_.State() == IdentifyState::DONE ||
        system_identify_.State() == IdentifyState::ABORTED) {
      identify_relax_lock_ = true;
      SetMode(GimbalEvent::SET_MODE_RELAX);
    }
  }

  void SetMode(GimbalEvent event) {
    if (event == GimbalEvent::SET_MODE_AUTOPATROL &&
        (cmd_.GetGimbalCtrlMode() != CMD::Mode::CMD_AUTO_CTRL ||
         !PARAM.patrol.big_yaw_euler_topic_name ||
         !PARAM.patrol.big_yaw_gyro_topic_name ||
         !std::isfinite(PARAM.patrol.big_yaw_period_s) ||
         PARAM.patrol.big_yaw_period_s <= 0.0f ||
         !std::isfinite(PARAM.patrol.small_yaw_amplitude_rad) ||
         PARAM.patrol.small_yaw_amplitude_rad <= 0.0f ||
         !std::isfinite(PARAM.patrol.small_yaw_period_s) ||
         PARAM.patrol.small_yaw_period_s <= 0.0f ||
         !std::isfinite(PARAM.patrol.pitch_amplitude_rad) ||
         !std::isfinite(PARAM.patrol.pitch_period_s) ||
         PARAM.patrol.pitch_period_s <= 0.0f)) {
      return;
    }
    if (identify_relax_lock_ && event != GimbalEvent::SET_MODE_RELAX) {
      return;
    }
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
    if (event == current_mode_) {
      return;
    }
    if (event == GimbalEvent::SET_MODE_IDENTIFY &&
        !PARAM.system_identify.enabled) {
      event = GimbalEvent::SET_MODE_RELAX;
    }
    current_mode_ = event;
    auto_aim_yaw_ = {};
    topic_auto_aim_yaw_.Publish(auto_aim_yaw_);

    uint8_t vision_task = 0U;
    switch (event) {
      case GimbalEvent::SET_MODE_RELAX:
        if (!big_yaw_relaxed_) {
          motor_yaw_->Disable();
          big_yaw_relaxed_ = true;
        }
        motor_pit_->Disable();
        motor_syaw_->Disable();
        manual_control_.Reset();
        auto_aim_control_.Reset();
        target_world_pitch_ = 0.0f;
        target_world_yaw_ = 0.0f;
        target_yaw_rate_ref_ = 0.0f;
        target_yaw_accel_ref_ = 0.0f;
        target_initialized_ = false;
        break;
      case GimbalEvent::SET_MODE_COMMON:
      case GimbalEvent::SET_VISION_IDLE:
      default:
        target_world_pitch_ = euler_.Pitch();
        target_world_yaw_ = euler_.Yaw();
        manual_control_.Reset();
        auto_aim_control_.Reset();
        target_yaw_rate_ref_ = 0.0f;
        target_yaw_accel_ref_ = 0.0f;
        target_initialized_ = false;
        break;
      case GimbalEvent::SET_MODE_AUTOPATROL:
        patrol_elapsed_s_ = 0.0f;
        ai_gimbal_reference_active_ = false;
        target_world_pitch_ = 0.0f;
        target_world_yaw_ = euler_.Yaw();
        manual_control_.Reset();
        auto_aim_control_.Reset();
        target_yaw_rate_ref_ = 0.0f;
        target_yaw_accel_ref_ = 0.0f;
        target_initialized_ = false;
        break;
      case GimbalEvent::SET_MODE_IDENTIFY:
        target_world_pitch_ = euler_.Pitch();
        target_world_yaw_ = euler_.Yaw();
        manual_control_.Reset();
        auto_aim_control_.Reset();
        target_yaw_rate_ref_ = 0.0f;
        target_yaw_accel_ref_ = 0.0f;
        target_initialized_ = false;
        system_identify_.Begin();
        break;
      case GimbalEvent::SET_VISION_AUTO_AIM:
        target_world_pitch_ = euler_.Pitch();
        target_world_yaw_ = euler_.Yaw();
        manual_control_.Reset();
        auto_aim_control_.Reset();
        target_yaw_rate_ref_ = 0.0f;
        target_yaw_accel_ref_ = 0.0f;
        target_initialized_ = false;
        vision_task = 1U;
        break;
      case GimbalEvent::SET_VISION_SMALL_BUFF:
        target_world_pitch_ = euler_.Pitch();
        target_world_yaw_ = euler_.Yaw();
        manual_control_.Reset();
        auto_aim_control_.Reset();
        target_yaw_rate_ref_ = 0.0f;
        target_yaw_accel_ref_ = 0.0f;
        target_initialized_ = false;
        vision_task = 2U;
        break;
      case GimbalEvent::SET_VISION_BIG_BUFF:
        target_world_pitch_ = euler_.Pitch();
        target_world_yaw_ = euler_.Yaw();
        manual_control_.Reset();
        auto_aim_control_.Reset();
        target_yaw_rate_ref_ = 0.0f;
        target_yaw_accel_ref_ = 0.0f;
        target_initialized_ = false;
        vision_task = 3U;
        break;
    }
    topic_vision_task_.Publish(vision_task);
  }
};

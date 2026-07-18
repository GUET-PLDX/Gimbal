#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-Gimbal.hpp}"
MODE="${2:-all}"
STATE_HEADER="$(dirname "${BASH_SOURCE[0]}")/../../Chassis/ChassisMotionState.hpp"

need() {
  rg -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

need_multiline() {
  rg -U -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

need_before() {
  local first_line second_line
  first_line="$(rg -n -m1 -- "$1" "$HEADER" | cut -d: -f1)"
  second_line="$(rg -n -m1 -- "$2" "$HEADER" | cut -d: -f1)"
  if [[ -z "$first_line" || -z "$second_line" || "$first_line" -ge "$second_line" ]]; then
    echo "misordered: $3" >&2
    exit 1
  fi
}

need_count() {
  local actual
  actual="$(rg -o -- "$1" "$HEADER" | wc -l)"
  if [[ "$actual" -ne "$2" ]]; then
    echo "wrong count ($actual != $2): $3" >&2
    exit 1
  fi
}

need_set_mode() {
  sed -n '/void SetMode(GimbalEvent gimbal_event)/,/^  }$/p' "$HEADER" |
    rg -U -q -- "$1" || { echo "missing: $2" >&2; exit 1; }
}

need_set_mode_count() {
  local actual
  actual="$(sed -n '/void SetMode(GimbalEvent gimbal_event)/,/^  }$/p' "$HEADER" |
    rg -o -- "$1" | wc -l)"
  if [[ "$actual" -ne "$2" ]]; then
    echo "wrong SetMode count ($actual != $2): $3" >&2
    exit 1
  fi
}

need_set_mode_before() {
  local set_mode first_line second_line
  set_mode="$(sed -n '/void SetMode(GimbalEvent gimbal_event)/,/^  }$/p' "$HEADER")"
  first_line="$(rg -n -m1 -- "$1" <<<"$set_mode" | cut -d: -f1)"
  second_line="$(rg -n -m1 -- "$2" <<<"$set_mode" | cut -d: -f1)"
  if [[ -z "$first_line" || -z "$second_line" ||
        "$first_line" -ge "$second_line" ]]; then
    echo "misordered in SetMode: $3" >&2
    exit 1
  fi
}

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
    exit 1
  fi
}

need_state() {
  rg -q -- "$1" "$STATE_HEADER" || {
    echo "missing: $2" >&2
    exit 1
  }
}

need_state '#include <cstdint>' 'ChassisMotionState fixed-width integer include'
need_state 'enum class ChassisMotionMode : uint8_t \{ NON_ROTOR, ROTOR \};' \
  'typed chassis motion mode enum'
need_state 'struct ChassisMotionState' 'semantic chassis motion state type'
need_state 'float yaw_rate_rad_s = 0\.0f' 'zero default chassis yaw rate'
need_state 'bool yaw_rate_valid = false' 'invalid default chassis yaw rate'
need_state 'bool online = false' 'offline default chassis link state'
need_state 'ChassisMotionMode mode = ChassisMotionMode::NON_ROTOR' \
  'non-rotor default chassis mode'

forbid 'CONTROL_DT_(MIN|MAX)|dt_valid_' 'control period validity guard'
forbid 'IMU_TIMEOUT_US|imu_online_|euler_received_|gyro_received_' \
  'IMU freshness guard'
need 'last_pit_angle_loop_omega_' 'Pitch angle-loop history'
need 'last_yaw_angle_loop_omega_' 'Yaw angle-loop history'
need 'pid_pit_omega_\.SetFeedForward' 'Pitch LibXR feedforward'
need 'pid_yaw_omega_\.SetFeedForward' 'Yaw LibXR feedforward'
need_multiline \
  '(?s)void Control\(\) \{.*float pit_output = 0\.0f;.*float yaw_output = 0\.0f;.*Solve\(pit_output, yaw_output\);' \
  'Control owns current-cycle torque outputs'
need 'void Solve\(float& pit_output, float& yaw_output\)' \
  'Solve writes current-cycle torque outputs by reference'
forbid 'float pit_output_ =|float yaw_output_ =' \
  'persistent control output members'
need_multiline \
  '-\s*this->pit_lc_ \* sinf\(euler_\.Pitch\(\) \+ this->pit_theta_\)' \
  'unchanged Pitch gravity formula'
forbid 'target_.*omega.*last_.*omega' 'derivative of total target omega'
forbid 'SleepUntil' 'SleepUntil scheduling'
forbid 'Telemetry' 'Telemetry structure'
forbid 'float torque_|this->torque_' \
  'duplicate Pitch gravity feedforward debug cache'

need_set_mode \
  'if \(gimbal_event == current_mode_\) \{\s*return;\s*\}' \
  'same-mode transition returns without resetting control state'
need_set_mode \
  '(?s)current_mode_ == GimbalEvent::SET_MODE_COMMON.*gimbal_event == GimbalEvent::SET_MODE_LOW_SENSITIVITY.*current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY.*gimbal_event == GimbalEvent::SET_MODE_COMMON.*current_mode_ = gimbal_event;\s*return;' \
  'COMMON and LOW_SENSITIVITY switch without resetting control state'
need_set_mode_before 'if \(gimbal_event == current_mode_\)' \
  'pid_pit_omega_\.SetFeedForward\(0\.0f\)' \
  'same-mode return precedes feedforward and history cleanup'
need_set_mode_before \
  'if \(\(current_mode_ == GimbalEvent::SET_MODE_COMMON' \
  'pid_pit_omega_\.SetFeedForward\(0\.0f\)' \
  'COMMON and LOW_SENSITIVITY return precedes feedforward and history cleanup'
need_set_mode \
  '(?s)if \(\(current_mode_ == GimbalEvent::SET_MODE_COMMON.*current_mode_ = gimbal_event;\s*return;\s*\}\s*current_mode_ = gimbal_event;\s*pid_pit_omega_\.SetFeedForward\(0\.0f\);\s*pid_yaw_omega_\.SetFeedForward\(0\.0f\);\s*last_pit_angle_loop_omega_ = 0\.0f;\s*last_yaw_angle_loop_omega_ = 0\.0f;' \
  'general mode assignment follows quick returns and precedes shared cleanup'
need_set_mode_count 'pid_pit_omega_\.SetFeedForward\(0\.0f\)' 1 \
  'Pitch feedforward cleanup is shared'
need_set_mode_count 'pid_yaw_omega_\.SetFeedForward\(0\.0f\)' 1 \
  'Yaw feedforward cleanup is shared'
need_set_mode_count 'last_pit_angle_loop_omega_ = 0\.0f' 1 \
  'Pitch angle-loop history cleanup is shared'
need_set_mode_count 'last_yaw_angle_loop_omega_ = 0\.0f' 1 \
  'Yaw angle-loop history cleanup is shared'
need_set_mode_count 'pid_pit_angle_\.Reset\(\)' 1 \
  'Pitch angle PID reset is shared by valid mode transitions'
need_set_mode_count 'pid_pit_omega_\.Reset\(\)' 1 \
  'Pitch speed PID reset is shared by valid mode transitions'
need_set_mode_count 'pid_yaw_angle_\.Reset\(\)' 1 \
  'Yaw angle PID reset is shared by valid mode transitions'
need_set_mode_count 'pid_yaw_omega_\.Reset\(\)' 1 \
  'Yaw speed PID reset is shared by valid mode transitions'
need_set_mode_count 'target_yaw_dot_ = 0\.0f' 1 \
  'Yaw target velocity cleanup is shared by valid mode transitions'
need_set_mode_count 'target_yaw_ddot_ = 0\.0f' 1 \
  'Yaw target acceleration cleanup is shared by valid mode transitions'
need_set_mode_count 'target_pit_dot_ = 0\.0f' 1 \
  'Pitch target velocity cleanup is shared by valid mode transitions'
need_set_mode_count 'target_pit_ddot_ = 0\.0f' 1 \
  'Pitch target acceleration cleanup is shared by valid mode transitions'
need_set_mode \
  'const bool RELAX =\s*gimbal_event == GimbalEvent::SET_MODE_RELAX;' \
  'RELAX transition classification'
need_set_mode \
  'const bool TRACKING_MODE =\s*gimbal_event == GimbalEvent::SET_MODE_COMMON \|\|\s*gimbal_event == GimbalEvent::SET_MODE_AUTOPATROL \|\|\s*gimbal_event == GimbalEvent::SET_MODE_LOW_SENSITIVITY;' \
  'tracking-mode transition classification'
need_set_mode \
  'if \(!RELAX && !TRACKING_MODE\) \{\s*return;\s*\}' \
  'unknown events skip PID and target derivative initialization'
need_set_mode \
  'if \(!RELAX && !TRACKING_MODE\) \{\s*return;\s*\}\s*pid_pit_angle_\.Reset\(\);\s*pid_pit_omega_\.Reset\(\);\s*pid_yaw_angle_\.Reset\(\);\s*pid_yaw_omega_\.Reset\(\);\s*target_yaw_dot_ = 0\.0f;\s*target_yaw_ddot_ = 0\.0f;\s*target_pit_dot_ = 0\.0f;\s*target_pit_ddot_ = 0\.0f;' \
  'valid modes share one unconditional PID and target derivative initialization block'
need_set_mode \
  'if \(RELAX\) \{\s*motor_yaw_->Disable\(\);\s*motor_pit_->Disable\(\);\s*target_pit_cmd_ = 0\.0f;\s*target_yaw_cmd_ = 0\.0f;\s*return;\s*\}\s*target_pit_cmd_ = euler_\.Pitch\(\);\s*target_yaw_cmd_ = euler_\.Yaw\(\);' \
  'RELAX returns after clearing targets before tracking modes anchor attitude'
need_set_mode \
  'if \(gimbal_event == GimbalEvent::SET_MODE_AUTOPATROL\) \{\s*patrol_start_time = LibXR::Timebase::GetMilliseconds\(\);\s*\}' \
  'AUTOPATROL records its start time'

if [[ "$MODE" != "core" ]]; then
  need 'target_yaw_dot_ = YAW_OPERATOR_RATE' 'manual Yaw rate feedforward'
  need 'target_pit_dot_ = PIT_OPERATOR_RATE' 'manual Pitch rate feedforward'
fi

need 'rotor_ff_enabled: false' 'default-disabled rotor feedforward manifest'
forbid '  - pldx/Referee|#include "Referee.hpp"|Referee\* referee|referee_|referee:' \
  'unused Referee interface dependency'
forbid '#include <cstdlib>|#include <cstring>|#define UI_GIMBAL_LAYER' \
  'unused headers and UI macro'
need '#include "ChassisMotionState.hpp"' 'shared chassis motion state contract include'
need 'FindOrCreate<ChassisMotionState>' 'typed chassis motion state Topic pre-creation'
need 'CHASSIS_MOTION_STATE_TOPIC_NAME' 'shared chassis motion state Topic name'
need 'CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER' 'shared chassis motion state Topic publisher policy'
need 'ASyncSubscriber<ChassisMotionState>' 'typed chassis motion state subscriber'
forbid 'ASyncSubscriber<float>[^;]*chassis_gyro_z' \
  'raw chassis gyro subscriber'
forbid 'ASyncSubscriber<uint32_t>[^;]*dualboard_chassis_mode' \
  'raw chassis mode subscriber'
forbid 'CHASSIS_MODE_ROTOR' 'numeric ROTOR protocol value'
forbid 'chassis_gyro_z_|dualboard_chassis_mode_' 'raw chassis state members'
need 'rotor_ff_enabled_' 'ROTOR feature flag member'
forbid 'chassis_rotor_active' 'second ROTOR Topic'
forbid 'rotor_weight' 'ROTOR fade weight'
forbid 'chassis_alpha_z' 'chassis angular acceleration'
forbid 'rotor_accel_k' 'chassis acceleration gain'
forbid 'rotor_ff_active_' 'ROTOR telemetry mirror'

need_multiline \
  'thread_priority: LibXR::Thread::Priority::MEDIUM\r?\n  - rotor_ff_enabled: false' \
  'rotor feedforward manifest option appended after thread priority'
need_multiline \
  'LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,\s*bool rotor_ff_enabled = false,\s*YawLqrEso::Config yaw_lqr_eso = \{\}\)' \
  'AI Yaw config appended after thread priority'
need_multiline \
  'ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber\s*\(\s*"gimbal_euler"\s*\)' \
  'Euler subscriber uses fixed Gimbal Topic name'
need_multiline \
  'ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber\s*\(\s*"gimbal_gyro"\s*\)' \
  'gyro subscriber uses fixed Gimbal Topic name'
forbid 'euler_topic_name|gyro_topic_name' 'configurable Gimbal IMU Topic names'
need 'bool rotor_ff_enabled_ = false' 'default-disabled feature flag member'
need 'ChassisMotionState chassis_motion_state_\{\}' \
  'default-initialized semantic chassis motion state member'

need_before 'FindOrCreate<ChassisMotionState>' 'thread_\.Create' \
  'chassis motion state Topic must exist before the Gimbal thread starts'
need_multiline \
  'ASyncSubscriber<ChassisMotionState>\s+chassis_motion_state_suber\s*\(\s*LibXR::Topic\(gimbal->chassis_motion_state_topic_\)\s*\)' \
  'typed chassis motion state subscriber constructed from its pre-created handle'
need_multiline \
  '(?s)chassis_motion_state_suber\.StartWaiting\(\);.*while \(true\).*chassis_motion_state_suber\.Available\(\).*chassis_motion_state_\s*=\s*chassis_motion_state_suber\.GetData\(\);.*chassis_motion_state_suber\.StartWaiting\(\);' \
  'latest typed chassis motion state Topic polling and re-arm'

need_multiline \
  'const bool ROTOR_FF_ACTIVE =\s*rotor_ff_enabled_ &&\s*chassis_motion_state_\.online &&\s*chassis_motion_state_\.yaw_rate_valid &&\s*chassis_motion_state_\.mode == ChassisMotionMode::ROTOR;' \
  'semantic online-valid ROTOR feature gate'
need_multiline \
  'const float YAW_MOTOR_OMEGA_REF =\s*ROTOR_FF_ACTIVE\s*\? TARGET_YAW_OMEGA - chassis_motion_state_\.yaw_rate_rad_s\s*: TARGET_YAW_OMEGA;' \
  'subtractive ROTOR-relative Yaw resistance reference'
need_multiline \
  'const float YAW_FEEDFORWARD =\s*j_yaw_ \* YAW_ALPHA \+ yaw_k_ \* YAW_MOTOR_OMEGA_REF;' \
  'relative reference used only by existing yaw resistance feedforward'
need 'pid_yaw_omega_\.Calculate\(TARGET_YAW_OMEGA, gyro_data_\.z\(\), dt_\)' \
  'unchanged Yaw speed-loop target'
need_count 'ROTOR_FF_ACTIVE' 2 'Solve-local ROTOR activation state'

echo 'PASS: Gimbal core static regression checks'

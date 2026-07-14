#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-Gimbal.hpp}"
MODE="${2:-all}"

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

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
    exit 1
  fi
}

need 'CONTROL_DT_MIN = 0\.0005f' 'minimum dt guard'
need 'CONTROL_DT_MAX = 0\.02f' 'maximum dt guard'
need 'dt_ > CONTROL_DT_MIN && dt_ <= CONTROL_DT_MAX' 'strict dt validity interval'
need 'IMU_TIMEOUT_US = 50000U' 'single IMU timeout'
need 'euler_received_' 'Euler received state'
need 'gyro_received_' 'gyro received state'
need 'last_euler_update_' 'Euler timestamp'
need 'last_gyro_update_' 'gyro timestamp'
need 'last_pit_angle_loop_omega_' 'Pitch angle-loop history'
need 'last_yaw_angle_loop_omega_' 'Yaw angle-loop history'
need 'pid_pit_omega_\.SetFeedForward' 'Pitch LibXR feedforward'
need 'pid_yaw_omega_\.SetFeedForward' 'Yaw LibXR feedforward'
need 'pit_output_' 'real Pitch output member'
need 'yaw_output_' 'real Yaw output member'
need '-this->pit_lc_ \* sinf\(euler_\.Pitch\(\) \+ this->pit_theta_\)' 'unchanged Pitch gravity formula'
forbid 'target_.*omega.*last_.*omega' 'derivative of total target omega'
forbid 'dt_ >= CONTROL_DT_MIN' 'inclusive lower dt guard'
forbid 'SleepUntil' 'SleepUntil scheduling'
forbid 'Telemetry' 'Telemetry structure'

if [[ "$MODE" != "core" ]]; then
  need 'target_yaw_dot_ = YAW_OPERATOR_RATE' 'manual Yaw rate feedforward'
  need 'target_pit_dot_ = PIT_OPERATOR_RATE' 'manual Pitch rate feedforward'
fi

need 'rotor_ff_enabled: false' 'default-disabled rotor feedforward manifest'
need '  - pldx/Referee' 'Referee manifest dependency'
need 'FindOrCreate<float>' 'chassis gyro Topic pre-creation'
need '"chassis_gyro_z", nullptr, false' 'single-publisher chassis gyro Topic'
need 'FindOrCreate<uint32_t>' 'chassis mode Topic pre-creation'
need '"dualboard_chassis_mode", nullptr, true' 'multi-publisher chassis mode Topic'
need 'CHASSIS_MODE_ROTOR = 2U' 'ROTOR protocol value'
need 'chassis_gyro_z_' 'chassis gyro control member'
need 'dualboard_chassis_mode_' 'requested chassis mode member'
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
  'LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,\s*bool rotor_ff_enabled = false,\s*bool ai_yaw_lqr_eso_enable = false,\s*YawLqrEso::Config yaw_lqr_eso = \{\},\s*const char\* euler_topic_name = "ahrs_euler",\s*const char\* gyro_topic_name = "bmi088_gyro"\)' \
  'AI Yaw options and compatible IMU Topic defaults appended in order'
need 'const char\* euler_topic_name_' 'stored Euler Topic name'
need 'const char\* gyro_topic_name_' 'stored gyro Topic name'
need_multiline \
  'ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber\s*\(\s*gimbal->euler_topic_name_\s*\)' \
  'Euler subscriber uses configured Topic name'
need_multiline \
  'ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber\s*\(\s*gimbal->gyro_topic_name_\s*\)' \
  'gyro subscriber uses configured Topic name'
forbid 'ASyncSubscriber<LibXR::EulerAngle<float>>[^;]*"gimbal_euler"' \
  'fixed Gimbal Euler subscriber Topic'
forbid 'ASyncSubscriber<Eigen::Matrix<float, 3, 1>>[^;]*"gimbal_gyro"' \
  'fixed Gimbal gyro subscriber Topic'
need 'bool rotor_ff_enabled_ = false' 'default-disabled feature flag member'
need 'float chassis_gyro_z_ = 0\.0f' 'zero-initialized chassis gyro member'
need 'uint32_t dualboard_chassis_mode_ = 0U' \
  'RELAX-initialized requested chassis mode member'

need_before 'FindOrCreate<float>' 'thread_\.Create' \
  'chassis gyro Topic must exist before the Gimbal thread starts'
need_before 'FindOrCreate<uint32_t>' 'thread_\.Create' \
  'chassis mode Topic must exist before the Gimbal thread starts'
need_multiline \
  'ASyncSubscriber<float> chassis_gyro_z_suber\s*\(\s*LibXR::Topic\(gimbal->chassis_gyro_z_topic_\)\s*\)' \
  'chassis gyro subscriber constructed from its pre-created handle'
need_multiline \
  'ASyncSubscriber<uint32_t> dualboard_chassis_mode_suber\s*\(\s*LibXR::Topic\(gimbal->dualboard_chassis_mode_topic_\)\s*\)' \
  'chassis mode subscriber constructed from its pre-created handle'
forbid 'ASyncSubscriber<float>[^;]*"chassis_gyro_z"' \
  'name-based chassis gyro subscriber construction'
forbid 'ASyncSubscriber<uint32_t>[^;]*"dualboard_chassis_mode"' \
  'name-based chassis mode subscriber construction'
need_multiline \
  '(?s)chassis_gyro_z_suber\.StartWaiting\(\);.*while \(true\).*chassis_gyro_z_suber\.Available\(\).*chassis_gyro_z_ = chassis_gyro_z_suber\.GetData\(\);.*chassis_gyro_z_suber\.StartWaiting\(\);' \
  'latest chassis gyro Topic polling and re-arm'
need_multiline \
  '(?s)dualboard_chassis_mode_suber\.StartWaiting\(\);.*while \(true\).*dualboard_chassis_mode_suber\.Available\(\).*dualboard_chassis_mode_\s*=\s*dualboard_chassis_mode_suber\.GetData\(\);.*dualboard_chassis_mode_suber\.StartWaiting\(\);' \
  'latest chassis mode Topic polling and re-arm'

need_multiline \
  'const bool ROTOR_FF_ACTIVE =\s*rotor_ff_enabled_ &&\s*dualboard_chassis_mode_ == CHASSIS_MODE_ROTOR;' \
  'exact requested-ROTOR feature gate'
need_multiline \
  'const float YAW_MOTOR_OMEGA_REF =\s*ROTOR_FF_ACTIVE \? TARGET_YAW_OMEGA - chassis_gyro_z_\s*: TARGET_YAW_OMEGA;' \
  'subtractive ROTOR-relative Yaw resistance reference'
need_multiline \
  'const float YAW_FEEDFORWARD =\s*j_yaw_ \* YAW_ALPHA \+ yaw_k_ \* YAW_MOTOR_OMEGA_REF;' \
  'relative reference used only by existing yaw resistance feedforward'
need 'pid_yaw_omega_\.Calculate\(TARGET_YAW_OMEGA, gyro_data_\.z\(\), dt_\)' \
  'unchanged Yaw speed-loop target'
need_count 'ROTOR_FF_ACTIVE' 2 'Solve-local ROTOR activation state'

echo 'PASS: Gimbal core static regression checks'

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "Motor.hpp"

namespace Pldx {

/// Topic carried over the navigation USB link. The sample time is the LibXR
/// packet header timestamp (48-bit microseconds, MCU Timebase), not a payload field.
inline constexpr char YAW_JOINT_FEEDBACK_TOPIC[] = "nav_yaw_joint";
inline constexpr uint64_t YAW_JOINT_TIMESTAMP_MASK = (1ULL << 48U) - 1U;

/// Big Yaw joint state from ONE motor feedback frame.
/// angle_rad: rotating end relative to the chassis, motor angle minus yaw_zero,
///   after the motor `reverse` setting, positive counter-clockwise seen from above.
/// velocity_rad_s: rate of angle_rad (same sign), decoded from the same CAN frame.
struct [[gnu::packed]] YawJointFeedback {
  float angle_rad{};
  float velocity_rad_s{};
};
static_assert(sizeof(YawJointFeedback) == 8U);
static_assert(std::is_trivially_copyable_v<YawJointFeedback>);

/// Builds a publishable sample. Returns false when the motor frame is not a
/// fresh, error-free sample. `timestamp_us` is the CAN reception time of the
/// frame that produced both fields, truncated to the 48-bit packet counter.
inline bool MakeYawJointFeedback(const Motor::Feedback& feedback, bool online,
                                 float yaw_zero_rad, uint16_t& last_sequence,
                                 YawJointFeedback& out, uint64_t& timestamp_us) {
  if (!online || feedback.error_id != 0U || feedback.received_time_us == 0U ||
      feedback.sequence == last_sequence) {
    return false;
  }
  const float angle = feedback.abs_angle - yaw_zero_rad;
  if (!std::isfinite(angle) || !std::isfinite(feedback.omega)) return false;
  last_sequence = feedback.sequence;
  out = {angle, feedback.omega};
  timestamp_us = feedback.received_time_us & YAW_JOINT_TIMESTAMP_MASK;
  return true;
}

}  // namespace Pldx

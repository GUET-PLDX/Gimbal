#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "cycle_value.hpp"
#include "transform.hpp"

// Paper conventions: right-handed ZYX, column vectors, R_BG = Rz(yaw)
// Ry(pitch).
namespace GimbalKinematics {

inline constexpr float PI = static_cast<float>(LibXR::PI);

struct Position {
  float yaw = 0.0f;
  float pitch = 0.0f;
};

struct PitchLimits {
  float min = -PI / 2.0f;
  float max = PI / 2.0f;
};

struct Config {
  float yaw_direction{1.0f};
  float pitch_direction{-1.0f};
  float reference_cutoff_hz{30.0f};
};

// Same half-turn convention as LibXR::CycleValue / the existing Yaw SMC:
// [-pi, pi). This is a target increment, whereas SMC uses feedback - target.
inline float ShortestYawDelta(float target, float previous) {
  return LibXR::CycleValue<float>(target) - previous;
}

// Direct-drive mapping AFTER the motor driver has applied its reverse setting.
// direction must be +1 or -1. Positions, rates and torque share this mapping.
struct AxisMapping {
  float zero;
  float direction;

  float Position(float encoder) const {
    return direction * ShortestYawDelta(encoder, zero);
  }
  float Velocity(float encoder_omega) const {
    return direction * encoder_omega;
  }
  float MotorTorque(float joint_torque) const {
    return direction * joint_torque;
  }
  float JointTorque(float motor_torque) const {
    return direction * motor_torque;
  }
  PitchLimits Limits(float encoder_min, float encoder_max) const {
    if (encoder_min == 0.0f && encoder_max == 0.0f) return {};
    const float A = Position(encoder_min), B = Position(encoder_max);
    return {std::min(A, B), std::max(A, B)};
  }
};

struct Reference {
  Position position;
  Position velocity;
  Position acceleration;
};

// Derivatives describe the IK reference trajectory; measured velocity remains
// motor CAN feedback. Each first-order filter uses the actual controller dt.
class ReferenceTracker {
 public:
  explicit ReferenceTracker(float cutoff_hz) : cutoff_hz_(cutoff_hz) {}

  void Reset() {
    initialized_ = false;
    reference_ = {};
  }

  Reference Update(Position position, float dt) {
    if (!initialized_) {
      reference_ = {.position = position, .velocity = {}, .acceleration = {}};
      previous_position_ = position;
      initialized_ = true;
      return reference_;
    }
    if (!std::isfinite(dt) || dt <= 0.0f) return reference_;
    const float ALPHA =
        cutoff_hz_ > 0.0f ? -std::expm1(-2.0f * PI * cutoff_hz_ * dt) : 1.0f;
    const Position DELTA{ShortestYawDelta(position.yaw, previous_position_.yaw),
                         position.pitch - previous_position_.pitch};
    const Position PREVIOUS_VELOCITY = reference_.velocity;
    reference_.position.yaw += DELTA.yaw;
    reference_.position.pitch = position.pitch;
    reference_.velocity.yaw +=
        ALPHA * (DELTA.yaw / dt - reference_.velocity.yaw);
    reference_.velocity.pitch +=
        ALPHA * (DELTA.pitch / dt - reference_.velocity.pitch);
    reference_.acceleration.yaw +=
        ALPHA * ((reference_.velocity.yaw - PREVIOUS_VELOCITY.yaw) / dt -
                 reference_.acceleration.yaw);
    reference_.acceleration.pitch +=
        ALPHA * ((reference_.velocity.pitch - PREVIOUS_VELOCITY.pitch) / dt -
                 reference_.acceleration.pitch);
    previous_position_ = position;
    return reference_;
  }

 private:
  float cutoff_hz_;
  bool initialized_ = false;
  Position previous_position_{};
  Reference reference_{};
};

// R_BG maps end-frame vectors into the base frame (paper equation 27).
inline Eigen::Matrix3f Forward(Position joint) {
  return LibXR::EulerAngle<float>(0.0f, joint.pitch, joint.yaw)
      .ToRotationMatrixZYX();
}

inline Eigen::Matrix3f InferBase(const Eigen::Matrix3f& end_to_world,
                                 Position joint) {
  return end_to_world * Forward(joint).transpose();
}

// World pitch is the paper's right-handed angle; application pitch is negated
// once, at the Gimbal command boundary.
inline Eigen::Vector3f WorldDirection(Position world) {
  return Forward(world).col(0);
}

inline Position Inverse(const Eigen::Matrix3f& base_to_world,
                        const Eigen::Vector3f& world_direction,
                        PitchLimits limits, float previous_yaw) {
  const Eigen::Vector3f BASE = base_to_world.transpose() * world_direction;
  const float HORIZONTAL = std::hypot(BASE.x(), BASE.y());
  // At the pole yaw is indeterminate. Preserve the preceding solution,
  // including when float rotations leave a small roundoff component in the
  // horizontal plane.
  return {
      .yaw = HORIZONTAL > 1e-6f ? std::atan2(BASE.y(), BASE.x()) : previous_yaw,
      .pitch = std::clamp(std::atan2(-BASE.z(), HORIZONTAL), limits.min,
                          limits.max)};
}

// Equivalent mg*r in the end frame, calibrated to the existing motor-space
// -pit_lc*sin(application_pitch + pit_theta) model on a level base.
inline float GravityTorque(const Eigen::Matrix3f& end_to_world, float pit_lc,
                           float pit_theta, float pitch_direction) {
  const Eigen::Vector3f ARM =
      pitch_direction * pit_lc *
      Eigen::Vector3f(std::sin(pit_theta), 0.0f, -std::cos(pit_theta));
  const Eigen::Vector3f AXIS = end_to_world.col(1);
  const Eigen::Vector3f LEVER = end_to_world * ARM;
  return -AXIS.dot(LEVER.cross(Eigen::Vector3f(0.0f, 0.0f, -1.0f)));
}

}  // namespace GimbalKinematics

#include <cassert>
#include <cmath>
#include <cstdio>

#include "TwoAxisIK.hpp"

namespace {

bool Near(float lhs, float rhs, float epsilon = 1e-4f) {
  return std::fabs(lhs - rhs) <= epsilon;
}

Eigen::Vector3f AimVector(float yaw, float pitch) {
  return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw),
          -std::sin(pitch)};
}

} // namespace

int main() {
  TwoAxisIK::Config config{};
  const LibXR::EulerAngle<float> level(0.0f, 0.0f, 0.0f);

  auto output = TwoAxisIK::Solve(LibXR::EulerAngle<float>(0.0f, 0.3f, 0.4f),
                                 level, 0.0f, 0.0f, config);
  assert(output.valid);
  assert(Near(output.yaw_rad, 0.4f));
  assert(Near(output.pitch_rad, 0.3f));
  assert((output.aim_vector_base - AimVector(0.4f, 0.3f)).norm() < 1e-4f);

  // A base roll couples world yaw and pitch; IK must recover joint angles.
  const float q_yaw = -0.25f;
  const float q_pitch = 0.18f;
  const LibXR::EulerAngle<float> imu(0.35f, 0.0f, 0.0f);
  const Eigen::Matrix3f r_wb = imu.ToRotationMatrixZYX();
  const Eigen::Matrix3f r_bg =
      LibXR::EulerAngle<float>(0.0f, q_pitch, q_yaw).ToRotationMatrixZYX();
  const Eigen::Vector3f view_world = r_wb * r_bg.col(0);
  LibXR::EulerAngle<float> target(
      0.9f,
      std::atan2(-view_world.z(), std::hypot(view_world.x(), view_world.y())),
      std::atan2(view_world.y(), view_world.x()));
  config.imu_at_end = false;
  output = TwoAxisIK::Solve(target, imu, q_yaw, q_pitch, config);
  assert(output.valid);
  assert(Near(LibXR::CycleValue<float>(output.yaw_rad) - q_yaw, 0.0f));
  assert(Near(output.pitch_rad, q_pitch));

  // The end-mounted IMU path reconstructs base orientation from encoder joints.
  config.imu_at_end = true;
  const Eigen::Matrix3f r_wg = r_wb * r_bg;
  const Eigen::Vector3f view_end = r_wg * Eigen::Vector3f::UnitX();
  LibXR::EulerAngle<float> target_end(
      0.0f, std::atan2(-view_end.z(), std::hypot(view_end.x(), view_end.y())),
      std::atan2(view_end.y(), view_end.x()));
  const LibXR::Quaternion<float> end_quaternion(r_wg);
  const LibXR::EulerAngle<float> end_imu(end_quaternion.ToEulerAngleZYX());
  output = TwoAxisIK::Solve(target_end, end_imu, q_yaw, q_pitch, config);
  assert(output.valid);
  assert(Near(LibXR::CycleValue<float>(output.yaw_rad) - q_yaw, 0.0f));
  assert(Near(output.pitch_rad, q_pitch));

  config = {};
  config.yaw_zero_rad = 1.2f;
  config.pitch_zero_rad = -0.4f;
  config.yaw_sign = -1.0f;
  config.pitch_sign = 1.0f;
  output = TwoAxisIK::Solve(LibXR::EulerAngle<float>(0.0f, 0.2f, 0.3f), level,
                            1.2f, -0.4f, config);
  assert(output.valid);
  assert(Near(LibXR::CycleValue<float>(output.yaw_rad) - (1.2f - 0.3f), 0.0f));
  assert(Near(output.pitch_rad, -0.4f + 0.2f));

  config.pitch_sign = -1.0f;
  output = TwoAxisIK::Solve(LibXR::EulerAngle<float>(0.0f, -0.1f, 0.0f), level,
                            1.2f, 0.4f, config);
  assert(output.valid);
  assert(Near(output.pitch_rad, 0.5f));

  config = {};
  config.yaw_min_rad = -0.2f;
  config.yaw_max_rad = 0.2f;
  config.pitch_min_rad = -0.1f;
  config.pitch_max_rad = 0.1f;
  output = TwoAxisIK::Solve(LibXR::EulerAngle<float>(0.0f, 0.4f, 0.7f), level,
                            0.0f, 0.0f, config);
  assert(output.valid && output.yaw_limited && output.pitch_limited);
  assert(Near(output.yaw_rad, 0.2f));
  assert(Near(output.pitch_rad, 0.1f));

  // The selected yaw equivalent is the nearest one inside the joint interval.
  config.yaw_min_rad = 5.5f;
  config.yaw_max_rad = 6.8f;
  config.imu_at_end = false;
  output = TwoAxisIK::Solve(LibXR::EulerAngle<float>(0.0f, 0.0f, -0.2f), level,
                            6.0f, 0.0f, config);
  assert(output.valid && !output.yaw_limited);
  assert(Near(output.yaw_rad, static_cast<float>(LibXR::TWO_PI) - 0.2f));

  config.yaw_min_rad = 0.0f;
  config.yaw_max_rad = 0.2f;
  output = TwoAxisIK::Solve(LibXR::EulerAngle<float>(0.0f, 0.0f, 1.0f), level,
                            0.1f, 0.0f, config);
  assert(output.valid && output.yaw_limited);
  assert(Near(output.yaw_rad, 0.2f));

  // At zenith/nadir, yaw is undefined; preserve the measured joint angle.
  config = {};
  output =
      TwoAxisIK::Solve(LibXR::EulerAngle<float>(
                           0.0f, -0.5f * static_cast<float>(LibXR::PI), 1.0f),
                       level, 0.37f, 0.0f, config);
  assert(output.valid);
  assert(Near(output.yaw_rad, 0.37f));

  output =
      TwoAxisIK::Solve(LibXR::EulerAngle<float>(0.0f, 0.0f, 0.0f), level, 0.0f,
                       0.0f, {.yaw_min_rad = 1.0f, .yaw_max_rad = -1.0f});
  assert(!output.valid);

  std::puts("PASS: two-axis gimbal IK");
}

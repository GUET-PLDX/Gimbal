#pragma once

#include <algorithm>
#include <cmath>

#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "transform.hpp"

class TwoAxisIK final {
public:
  // Joint angles are in the physical frame.  The encoder frame is related by
  // physical = sign * (encoder - zero).
  struct Config {
    bool imu_at_end{true};
    float yaw_sign{1.0f};
    float pitch_sign{1.0f};
    float yaw_zero_rad{};
    float pitch_zero_rad{};
    float yaw_min_rad{-static_cast<float>(LibXR::PI)};
    float yaw_max_rad{static_cast<float>(LibXR::PI)};
    float pitch_min_rad{-static_cast<float>(LibXR::PI / 2.0)};
    float pitch_max_rad{static_cast<float>(LibXR::PI / 2.0)};
  };

  struct Output {
    Eigen::Vector3f aim_vector_base{Eigen::Vector3f::Zero()};
    float yaw_rad{};
    float pitch_rad{};
    bool yaw_limited{};
    bool pitch_limited{};
    bool valid{};
  };

  static Output Solve(const LibXR::EulerAngle<float> &target_world,
                      const LibXR::EulerAngle<float> &imu_world,
                      float measured_yaw_rad, float measured_pitch_rad,
                      const Config &config) {
    Output output{};
    if (!std::isfinite(target_world.Yaw()) ||
        !std::isfinite(target_world.Pitch()) ||
        !std::isfinite(imu_world.Roll()) || !std::isfinite(imu_world.Pitch()) ||
        !std::isfinite(imu_world.Yaw()) || !std::isfinite(measured_yaw_rad) ||
        !std::isfinite(measured_pitch_rad) ||
        !std::isfinite(config.yaw_min_rad) ||
        !std::isfinite(config.yaw_max_rad) ||
        !std::isfinite(config.pitch_min_rad) ||
        !std::isfinite(config.pitch_max_rad) ||
        !std::isfinite(config.yaw_sign) || !std::isfinite(config.pitch_sign) ||
        !std::isfinite(config.yaw_zero_rad) ||
        !std::isfinite(config.pitch_zero_rad) ||
        std::fabs(std::fabs(config.yaw_sign) - 1.0f) > 1.0e-4f ||
        std::fabs(std::fabs(config.pitch_sign) - 1.0f) > 1.0e-4f ||
        config.yaw_min_rad >= config.yaw_max_rad ||
        config.pitch_min_rad >= config.pitch_max_rad) {
      return output;
    }

    const Eigen::Matrix3f rotation_world_gimbal =
        imu_world.ToRotationMatrixZYX();
    const float physical_yaw =
        config.yaw_sign *
        (LibXR::CycleValue<float>(measured_yaw_rad) - config.yaw_zero_rad);
    const float physical_pitch =
        config.pitch_sign *
        (LibXR::CycleValue<float>(measured_pitch_rad) - config.pitch_zero_rad);
    const Eigen::Matrix3f rotation_base_gimbal =
        LibXR::EulerAngle<float>(0.0f, physical_pitch, physical_yaw)
            .ToRotationMatrixZYX();
    const Eigen::Matrix3f rotation_world_base =
        config.imu_at_end
            ? rotation_world_gimbal * rotation_base_gimbal.transpose()
            : rotation_world_gimbal;

    const float target_yaw = target_world.Yaw();
    const float target_pitch = target_world.Pitch();
    const Eigen::Vector3f aim_world(
        std::cos(target_pitch) * std::cos(target_yaw),
        std::cos(target_pitch) * std::sin(target_yaw), -std::sin(target_pitch));
    output.aim_vector_base = rotation_world_base.transpose() * aim_world;
    if (!output.aim_vector_base.allFinite()) {
      return {};
    }

    const float horizontal =
        std::hypot(output.aim_vector_base.x(), output.aim_vector_base.y());
    const float yaw_raw = horizontal < 1.0e-5f
                              ? physical_yaw
                              : std::atan2(output.aim_vector_base.y(),
                                           output.aim_vector_base.x());
    const float pitch_raw = std::atan2(-output.aim_vector_base.z(), horizontal);

    const float two_pi = static_cast<float>(LibXR::TWO_PI);
    const float min_turn = std::ceil((config.yaw_min_rad - yaw_raw) / two_pi);
    const float max_turn = std::floor((config.yaw_max_rad - yaw_raw) / two_pi);
    const bool found_yaw_branch = min_turn <= max_turn;
    const float nearest_turn = std::round((physical_yaw - yaw_raw) / two_pi);
    const float yaw_candidate =
        found_yaw_branch
            ? yaw_raw + std::clamp(nearest_turn, min_turn, max_turn) * two_pi
            : yaw_raw;
    float physical_yaw_target = yaw_candidate;
    if (!found_yaw_branch) {
      const float min_error =
          std::fabs(LibXR::CycleValue<float>(config.yaw_min_rad) - yaw_raw);
      const float max_error =
          std::fabs(LibXR::CycleValue<float>(config.yaw_max_rad) - yaw_raw);
      physical_yaw_target = min_error < max_error   ? config.yaw_min_rad
                            : max_error < min_error ? config.yaw_max_rad
                            : std::fabs(config.yaw_min_rad - physical_yaw) <
                                    std::fabs(config.yaw_max_rad - physical_yaw)
                                ? config.yaw_min_rad
                                : config.yaw_max_rad;
    }
    const float physical_pitch_target =
        std::clamp(pitch_raw, config.pitch_min_rad, config.pitch_max_rad);
    output.yaw_rad =
        config.yaw_zero_rad + config.yaw_sign * physical_yaw_target;
    output.pitch_rad =
        config.pitch_zero_rad + config.pitch_sign * physical_pitch_target;
    output.yaw_limited = !found_yaw_branch;
    output.pitch_limited = physical_pitch_target != pitch_raw;
    output.valid =
        std::isfinite(output.yaw_rad) && std::isfinite(output.pitch_rad);
    return output;
  }
};

#pragma once

#include <cmath>

namespace PatrolTrajectory {
inline float PitchTarget(float center_rad, float amplitude_rad,
                         float angular_rate_rad_s, float elapsed_s) {
  constexpr float TWO_OVER_PI = 0.6366197723675814f;
  return center_rad + amplitude_rad * TWO_OVER_PI *
                          std::asin(std::sin(angular_rate_rad_s * elapsed_s));
}
}  // namespace PatrolTrajectory

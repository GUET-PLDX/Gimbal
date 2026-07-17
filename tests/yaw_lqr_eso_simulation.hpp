#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

struct ReferenceSample {
  double theta{};
  double omega{};
  double alpha{};
};

struct PlantState {
  double theta{};
  double omega{};
};

constexpr double deg_to_rad(double degrees) {
  return degrees * std::numbers::pi / 180.0;
}

constexpr double rad_to_deg(double radians) {
  return radians * 180.0 / std::numbers::pi;
}

inline double inertia_torque_peak(double plant_j, double frequency,
                                  double amplitude) {
  const double OMEGA = 2.0 * std::numbers::pi * frequency;
  return plant_j * amplitude * OMEGA * OMEGA;
}

inline double inertia_torque_rms(double plant_j, double frequency,
                                 double amplitude) {
  return inertia_torque_peak(plant_j, frequency, amplitude) / std::sqrt(2.0);
}

inline double torque_limited_amplitude_deg(double plant_j, double frequency,
                                           double torque_limit) {
  const double OMEGA = 2.0 * std::numbers::pi * frequency;
  return rad_to_deg(torque_limit / (plant_j * OMEGA * OMEGA));
}

inline ReferenceSample sine_reference(double amplitude, double frequency,
                                      double time) {
  const double OMEGA = 2.0 * std::numbers::pi * frequency;
  const double PHASE = OMEGA * time;
  return {.theta = amplitude * std::sin(PHASE),
          .omega = amplitude * OMEGA * std::cos(PHASE),
          .alpha = -amplitude * OMEGA * OMEGA * std::sin(PHASE)};
}

constexpr double minimum_jerk_scale(double ratio) {
  const double RATIO_SQUARED = ratio * ratio;
  const double RATIO_CUBED = RATIO_SQUARED * ratio;
  return RATIO_CUBED * (10.0 + ratio * (-15.0 + 6.0 * ratio));
}

constexpr double minimum_jerk_scale_first_derivative(double ratio) {
  const double RATIO_SQUARED = ratio * ratio;
  return RATIO_SQUARED * (30.0 + ratio * (-60.0 + 30.0 * ratio));
}

constexpr double minimum_jerk_scale_second_derivative(double ratio) {
  return ratio * (60.0 + ratio * (-180.0 + 120.0 * ratio));
}

inline ReferenceSample minimum_jerk_reference(double start_theta,
                                              double finish_theta,
                                              double duration, double time) {
  const double RATIO = time / duration;
  const double THETA_DELTA = finish_theta - start_theta;
  return {
      .theta = start_theta + THETA_DELTA * minimum_jerk_scale(RATIO),
      .omega =
          THETA_DELTA * minimum_jerk_scale_first_derivative(RATIO) / duration,
      .alpha = THETA_DELTA * minimum_jerk_scale_second_derivative(RATIO) /
               (duration * duration),
  };
}

class QuinticTrajectory final {
 public:
  QuinticTrajectory(ReferenceSample start, ReferenceSample finish,
                    double duration)
      : duration_(duration) {
    const double DURATION_SQUARED = duration * duration;
    const double DURATION_CUBED = DURATION_SQUARED * duration;
    const double DURATION_FOURTH = DURATION_CUBED * duration;
    const double DURATION_FIFTH = DURATION_FOURTH * duration;
    const double POSITION_REMAINDER =
        finish.theta - (start.theta + start.omega * duration +
                        0.5 * start.alpha * DURATION_SQUARED);
    const double VELOCITY_REMAINDER =
        finish.omega - (start.omega + start.alpha * duration);
    const double ACCELERATION_REMAINDER = finish.alpha - start.alpha;

    coefficients_[0] = start.theta;
    coefficients_[1] = start.omega;
    coefficients_[2] = 0.5 * start.alpha;
    coefficients_[3] = 10.0 * POSITION_REMAINDER / DURATION_CUBED -
                       4.0 * VELOCITY_REMAINDER / DURATION_SQUARED +
                       0.5 * ACCELERATION_REMAINDER / duration;
    coefficients_[4] = -15.0 * POSITION_REMAINDER / DURATION_FOURTH +
                       7.0 * VELOCITY_REMAINDER / DURATION_CUBED -
                       ACCELERATION_REMAINDER / DURATION_SQUARED;
    coefficients_[5] = 6.0 * POSITION_REMAINDER / DURATION_FIFTH -
                       3.0 * VELOCITY_REMAINDER / DURATION_FOURTH +
                       0.5 * ACCELERATION_REMAINDER / DURATION_CUBED;
  }

  ReferenceSample Sample(double time) const {
    const double TIME_SQUARED = time * time;
    const double TIME_CUBED = TIME_SQUARED * time;
    const double TIME_FOURTH = TIME_CUBED * time;
    const double TIME_FIFTH = TIME_FOURTH * time;
    return {
        .theta = coefficients_[0] + coefficients_[1] * time +
                 coefficients_[2] * TIME_SQUARED +
                 coefficients_[3] * TIME_CUBED +
                 coefficients_[4] * TIME_FOURTH + coefficients_[5] * TIME_FIFTH,
        .omega = coefficients_[1] + 2.0 * coefficients_[2] * time +
                 3.0 * coefficients_[3] * TIME_SQUARED +
                 4.0 * coefficients_[4] * TIME_CUBED +
                 5.0 * coefficients_[5] * TIME_FOURTH,
        .alpha = 2.0 * coefficients_[2] + 6.0 * coefficients_[3] * time +
                 12.0 * coefficients_[4] * TIME_SQUARED +
                 20.0 * coefficients_[5] * TIME_CUBED,
    };
  }

  double Duration() const { return duration_; }

 private:
  std::array<double, 6> coefficients_{};
  double duration_{};
};

inline PlantState propagate_exact_zoh(PlantState state, double plant_j,
                                      double plant_b, double torque,
                                      double disturbance, double dt) {
  const double INPUT = torque + disturbance;
  if (plant_b == 0.0) {
    const double ACCELERATION = INPUT / plant_j;
    state.theta += state.omega * dt + 0.5 * ACCELERATION * dt * dt;
    state.omega += ACCELERATION * dt;
  } else {
    const double A = plant_b / plant_j;
    const double Q = std::exp(-A * dt);
    const double OMEGA_SS = INPUT / plant_b;
    state.theta += OMEGA_SS * dt + (state.omega - OMEGA_SS) * (1.0 - Q) / A;
    state.omega = OMEGA_SS + (state.omega - OMEGA_SS) * Q;
  }
  return state;
}

inline double wrap_pi(double angle) {
  constexpr double TWO_PI = 2.0 * std::numbers::pi;
  double wrapped = std::fmod(angle + std::numbers::pi, TWO_PI);
  if (wrapped < 0.0) {
    wrapped += TWO_PI;
  }
  return wrapped - std::numbers::pi;
}

class LegacyYawAdapter final {
 public:
  void Reset() { last_angle_loop_ = 0.0; }

  double Calculate(ReferenceSample reference, double theta, double omega,
                   double dt) {
    const double ANGLE_LOOP = wrap_pi(reference.theta - theta);
    const double OMEGA_CMD = ANGLE_LOOP + reference.omega;
    const double ALPHA_CMD =
        (ANGLE_LOOP - last_angle_loop_) / dt + reference.alpha;
    const double TAU_RAW = (OMEGA_CMD - omega) + 0.03 * ALPHA_CMD;
    last_angle_loop_ = ANGLE_LOOP;
    return std::clamp(TAU_RAW, -2.223, 2.223);
  }

 private:
  double last_angle_loop_{};
};

struct WeightedValueSample {
  double value{};
  double dt{};
};

inline double weighted_rmse(std::span<const WeightedValueSample> samples) {
  double weighted_square_sum = 0.0;
  double total_time = 0.0;
  for (const WeightedValueSample& SAMPLE : samples) {
    if (!std::isfinite(SAMPLE.value) || !std::isfinite(SAMPLE.dt) ||
        SAMPLE.dt <= 0.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    weighted_square_sum += SAMPLE.value * SAMPLE.value * SAMPLE.dt;
    total_time += SAMPLE.dt;
  }
  if (!(total_time > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::sqrt(weighted_square_sum / total_time);
}

inline double weighted_absolute_p95(
    std::span<const WeightedValueSample> samples) {
  std::vector<WeightedValueSample> sorted_samples;
  sorted_samples.reserve(samples.size());
  double total_time = 0.0;
  for (const WeightedValueSample& SAMPLE : samples) {
    if (!std::isfinite(SAMPLE.value) || !std::isfinite(SAMPLE.dt) ||
        SAMPLE.dt <= 0.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    sorted_samples.push_back(
        {.value = std::fabs(SAMPLE.value), .dt = SAMPLE.dt});
    total_time += SAMPLE.dt;
  }
  if (!(total_time > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::sort(
      sorted_samples.begin(), sorted_samples.end(),
      [](const WeightedValueSample& LEFT, const WeightedValueSample& RIGHT) {
        return LEFT.value < RIGHT.value;
      });
  const double P95_TIME = 0.95 * total_time;
  double cumulative_time = 0.0;
  for (const WeightedValueSample& SAMPLE : sorted_samples) {
    cumulative_time += SAMPLE.dt;
    if (cumulative_time >= P95_TIME) {
      return SAMPLE.value;
    }
  }
  return sorted_samples.back().value;
}

struct LimitActivitySample {
  double dt{};
  bool soft_limit_active{};
  bool hard_limit_active{};
  bool slew_limit_active{};
};

struct ActiveTimeRatios {
  double soft_ratio{};
  double hard_ratio{};
  double soft_hard_union_ratio{};
  double slew_ratio{};
  bool valid{};
};

inline ActiveTimeRatios active_time_ratios(
    std::span<const LimitActivitySample> samples) {
  double total_time = 0.0;
  double soft_time = 0.0;
  double hard_time = 0.0;
  double soft_hard_union_time = 0.0;
  double slew_time = 0.0;
  for (const LimitActivitySample& SAMPLE : samples) {
    if (!std::isfinite(SAMPLE.dt) || SAMPLE.dt <= 0.0) {
      return {};
    }
    total_time += SAMPLE.dt;
    if (SAMPLE.soft_limit_active) {
      soft_time += SAMPLE.dt;
    }
    if (SAMPLE.hard_limit_active) {
      hard_time += SAMPLE.dt;
    }
    if (SAMPLE.soft_limit_active || SAMPLE.hard_limit_active) {
      soft_hard_union_time += SAMPLE.dt;
    }
    if (SAMPLE.slew_limit_active) {
      slew_time += SAMPLE.dt;
    }
  }
  if (!(total_time > 0.0)) {
    return {};
  }
  return {
      .soft_ratio = soft_time / total_time,
      .hard_ratio = hard_time / total_time,
      .soft_hard_union_ratio = soft_hard_union_time / total_time,
      .slew_ratio = slew_time / total_time,
      .valid = true,
  };
}

struct PhaseSample {
  double time{};
  double value{};
  double dt{};
  bool soft_limit_active{};
  bool hard_limit_active{};
};

struct PhaseFit {
  double offset{};
  double sine_coefficient{};
  double cosine_coefficient{};
  double gain{};
  double phase_lag{};
  double soft_hard_union_ratio{};
  bool valid{};
};

inline bool solve_three_by_three(std::array<std::array<double, 3>, 3> matrix,
                                 std::array<double, 3> right_hand_side,
                                 std::array<double, 3>& solution) {
  constexpr std::size_t SIZE = 3U;
  constexpr double PIVOT_TOLERANCE = 1.0e-14;
  for (std::size_t column = 0; column < SIZE; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1U; row < SIZE; ++row) {
      if (std::fabs(matrix[row][column]) > std::fabs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::fabs(matrix[pivot][column]) <= PIVOT_TOLERANCE) {
      return false;
    }
    if (pivot != column) {
      std::swap(matrix[pivot], matrix[column]);
      std::swap(right_hand_side[pivot], right_hand_side[column]);
    }

    const double PIVOT_VALUE = matrix[column][column];
    for (std::size_t entry = column; entry < SIZE; ++entry) {
      matrix[column][entry] /= PIVOT_VALUE;
    }
    right_hand_side[column] /= PIVOT_VALUE;

    for (std::size_t row = 0; row < SIZE; ++row) {
      if (row == column) {
        continue;
      }
      const double FACTOR = matrix[row][column];
      for (std::size_t entry = column; entry < SIZE; ++entry) {
        matrix[row][entry] -= FACTOR * matrix[column][entry];
      }
      right_hand_side[row] -= FACTOR * right_hand_side[column];
    }
  }
  solution = right_hand_side;
  return true;
}

inline PhaseFit fit_phase(std::span<const PhaseSample> samples,
                          double frequency, double reference_amplitude) {
  PhaseFit fit{};
  if (!std::isfinite(frequency) || !std::isfinite(reference_amplitude) ||
      frequency <= 0.0 || reference_amplitude <= 0.0) {
    return fit;
  }

  std::array<std::array<double, 3>, 3> normal_matrix{};
  std::array<double, 3> right_hand_side{};
  double total_time = 0.0;
  double soft_hard_union_time = 0.0;
  const double OMEGA = 2.0 * std::numbers::pi * frequency;
  for (const PhaseSample& SAMPLE : samples) {
    if (!std::isfinite(SAMPLE.time) || !std::isfinite(SAMPLE.value) ||
        !std::isfinite(SAMPLE.dt) || SAMPLE.dt <= 0.0) {
      return fit;
    }
    const std::array<double, 3> BASIS{{
        1.0,
        std::sin(OMEGA * SAMPLE.time),
        std::cos(OMEGA * SAMPLE.time),
    }};
    for (std::size_t row = 0; row < BASIS.size(); ++row) {
      right_hand_side[row] += SAMPLE.dt * BASIS[row] * SAMPLE.value;
      for (std::size_t column = 0; column < BASIS.size(); ++column) {
        normal_matrix[row][column] += SAMPLE.dt * BASIS[row] * BASIS[column];
      }
    }
    total_time += SAMPLE.dt;
    if (SAMPLE.soft_limit_active || SAMPLE.hard_limit_active) {
      soft_hard_union_time += SAMPLE.dt;
    }
  }
  if (!(total_time > 0.0)) {
    return fit;
  }

  fit.soft_hard_union_ratio = soft_hard_union_time / total_time;
  std::array<double, 3> coefficients{};
  if (!solve_three_by_three(normal_matrix, right_hand_side, coefficients)) {
    return fit;
  }

  fit.offset = coefficients[0];
  fit.sine_coefficient = coefficients[1];
  fit.cosine_coefficient = coefficients[2];
  fit.gain = std::hypot(fit.sine_coefficient, fit.cosine_coefficient) /
             reference_amplitude;
  fit.phase_lag = std::atan2(-fit.cosine_coefficient, fit.sine_coefficient);
  constexpr double SOFT_HARD_UNION_LIMIT = 0.01;
  constexpr double RATIO_ROUNDING_TOLERANCE =
      64.0 * std::numeric_limits<double>::epsilon();
  fit.valid = fit.soft_hard_union_ratio <
                  SOFT_HARD_UNION_LIMIT - RATIO_ROUNDING_TOLERANCE &&
              std::isfinite(fit.offset) && std::isfinite(fit.gain) &&
              std::isfinite(fit.phase_lag);
  return fit;
}

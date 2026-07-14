#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "yaw_lqr_eso_simulation.hpp"
#include "yaw_lqr_eso_test_support.hpp"

static void check_near_double(double actual, double expected, double tolerance,
                              const char* expression, int line) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::fprintf(stderr, "FAIL line %d: %s actual=%.17g expected=%.17g\n", line,
                 expression, actual, expected);
    ++yaw_test_failures;
  }
}

#define CHECK_NEAR_DOUBLE(ACTUAL, EXPECTED, TOLERANCE) \
  check_near_double((ACTUAL), (EXPECTED), (TOLERANCE), #ACTUAL, __LINE__)

static void test_physical_torque_bounds() {
  CHECK_NEAR(
      static_cast<float>(inertia_torque_peak(0.03, 3.0, deg_to_rad(10.0))),
      1.8603766f, 1.0e-5f);
  CHECK_NEAR(
      static_cast<float>(inertia_torque_rms(0.03, 3.0, deg_to_rad(10.0))),
      1.3154849f, 1.0e-5f);
  CHECK_NEAR(
      static_cast<float>(inertia_torque_peak(0.03, 5.0, deg_to_rad(10.0))),
      5.1677128f, 1.0e-5f);
  CHECK_NEAR(static_cast<float>(torque_limited_amplitude_deg(0.03, 5.0, 2.223)),
             4.30171f, 1.0e-4f);
  CHECK_NEAR(static_cast<float>(torque_limited_amplitude_deg(0.03, 5.0, 2.0)),
             3.87018f, 1.0e-4f);
  CHECK_NEAR(static_cast<float>(torque_limited_amplitude_deg(0.03, 3.0, 2.0)),
             10.75051f, 1.0e-4f);
}

static void test_sine_reference_derivatives() {
  constexpr double AMPLITUDE = 0.4;
  constexpr double FREQUENCY = 2.5;
  constexpr double OMEGA = 2.0 * std::numbers::pi * FREQUENCY;

  const ReferenceSample START = sine_reference(AMPLITUDE, FREQUENCY, 0.0);
  CHECK_NEAR_DOUBLE(START.theta, 0.0, 1.0e-14);
  CHECK_NEAR_DOUBLE(START.omega, AMPLITUDE * OMEGA, 1.0e-13);
  CHECK_NEAR_DOUBLE(START.alpha, 0.0, 1.0e-12);

  const ReferenceSample QUARTER_PERIOD =
      sine_reference(AMPLITUDE, FREQUENCY, 0.25 / FREQUENCY);
  CHECK_NEAR_DOUBLE(QUARTER_PERIOD.theta, AMPLITUDE, 1.0e-14);
  CHECK_NEAR_DOUBLE(QUARTER_PERIOD.omega, 0.0, 1.0e-13);
  CHECK_NEAR_DOUBLE(QUARTER_PERIOD.alpha, -AMPLITUDE * OMEGA * OMEGA, 1.0e-12);
}

static void test_minimum_jerk_boundary_conditions() {
  constexpr double START_THETA = -0.2;
  constexpr double FINISH_THETA = 0.3;
  constexpr double DURATION = 0.2;

  const ReferenceSample START =
      minimum_jerk_reference(START_THETA, FINISH_THETA, DURATION, 0.0);
  CHECK_NEAR_DOUBLE(START.theta, START_THETA, 1.0e-14);
  CHECK_NEAR_DOUBLE(START.omega, 0.0, 1.0e-14);
  CHECK_NEAR_DOUBLE(START.alpha, 0.0, 1.0e-14);

  const ReferenceSample FINISH =
      minimum_jerk_reference(START_THETA, FINISH_THETA, DURATION, DURATION);
  CHECK_NEAR_DOUBLE(FINISH.theta, FINISH_THETA, 1.0e-13);
  CHECK_NEAR_DOUBLE(FINISH.omega, 0.0, 1.0e-12);
  CHECK_NEAR_DOUBLE(FINISH.alpha, 0.0, 1.0e-11);

  const ReferenceSample MIDPOINT = minimum_jerk_reference(
      START_THETA, FINISH_THETA, DURATION, 0.5 * DURATION);
  CHECK_NEAR_DOUBLE(MIDPOINT.theta, 0.5 * (START_THETA + FINISH_THETA),
                    1.0e-14);
  CHECK_NEAR_DOUBLE(MIDPOINT.omega,
                    (FINISH_THETA - START_THETA) * 1.875 / DURATION, 1.0e-13);
  CHECK_NEAR_DOUBLE(MIDPOINT.alpha, 0.0, 1.0e-12);
}

static void test_general_quintic_boundary_conditions() {
  constexpr ReferenceSample START{.theta = 0.3, .omega = -0.4, .alpha = 1.2};
  constexpr ReferenceSample FINISH{.theta = -0.2, .omega = 0.6, .alpha = -0.7};
  constexpr double DURATION = 0.2;
  const QuinticTrajectory TRAJECTORY(START, FINISH, DURATION);

  const ReferenceSample ACTUAL_START = TRAJECTORY.Sample(0.0);
  CHECK_NEAR_DOUBLE(ACTUAL_START.theta, START.theta, 1.0e-13);
  CHECK_NEAR_DOUBLE(ACTUAL_START.omega, START.omega, 1.0e-13);
  CHECK_NEAR_DOUBLE(ACTUAL_START.alpha, START.alpha, 1.0e-13);

  const ReferenceSample ACTUAL_FINISH = TRAJECTORY.Sample(DURATION);
  CHECK_NEAR_DOUBLE(ACTUAL_FINISH.theta, FINISH.theta, 1.0e-11);
  CHECK_NEAR_DOUBLE(ACTUAL_FINISH.omega, FINISH.omega, 1.0e-10);
  CHECK_NEAR_DOUBLE(ACTUAL_FINISH.alpha, FINISH.alpha, 1.0e-9);
}

static void test_exact_zoh_plant() {
  constexpr PlantState INITIAL_UNDAMPED{.theta = 0.25, .omega = -0.4};
  const PlantState UNDAMPED =
      propagate_exact_zoh(INITIAL_UNDAMPED, 0.03, 0.0, 0.7, -0.1, 0.02);
  CHECK_NEAR_DOUBLE(UNDAMPED.theta, 0.246, 1.0e-14);
  CHECK_NEAR_DOUBLE(UNDAMPED.omega, 0.0, 1.0e-14);

  constexpr PlantState INITIAL_DAMPED{.theta = 0.1, .omega = -0.5};
  constexpr double J = 0.5;
  constexpr double B = 2.0;
  constexpr double INPUT = 3.0;
  constexpr double DT = 0.25;
  const double Q = std::exp(-(B / J) * DT);
  const double OMEGA_SS = INPUT / B;
  const double EXPECTED_THETA =
      INITIAL_DAMPED.theta + OMEGA_SS * DT +
      (INITIAL_DAMPED.omega - OMEGA_SS) * (1.0 - Q) / (B / J);
  const double EXPECTED_OMEGA =
      OMEGA_SS + (INITIAL_DAMPED.omega - OMEGA_SS) * Q;
  const PlantState DAMPED =
      propagate_exact_zoh(INITIAL_DAMPED, J, B, 2.5, 0.5, DT);
  CHECK_NEAR_DOUBLE(DAMPED.theta, EXPECTED_THETA, 1.0e-14);
  CHECK_NEAR_DOUBLE(DAMPED.omega, EXPECTED_OMEGA, 1.0e-14);
}

static void test_legacy_adapter_first_tick() {
  LegacyYawAdapter adapter;
  adapter.Reset();
  constexpr ReferenceSample REFERENCE{.theta = 0.1, .omega = 0.0, .alpha = 0.0};
  CHECK_NEAR_DOUBLE(adapter.Calculate(REFERENCE, 0.0, 0.0, 0.002), 1.6,
                    1.0e-13);
}

static void test_time_weighted_metrics() {
  constexpr std::array<WeightedValueSample, 2> RMSE_SAMPLES{{
      {.value = 1.0, .dt = 1.0},
      {.value = 3.0, .dt = 3.0},
  }};
  CHECK_NEAR_DOUBLE(weighted_rmse(RMSE_SAMPLES), std::sqrt(7.0), 1.0e-14);

  constexpr std::array<WeightedValueSample, 2> P95_SAMPLES{{
      {.value = -1.0, .dt = 0.96},
      {.value = 100.0, .dt = 0.04},
  }};
  CHECK_NEAR_DOUBLE(weighted_absolute_p95(P95_SAMPLES), 1.0, 1.0e-14);

  constexpr std::array<LimitActivitySample, 4> LIMIT_SAMPLES{{
      {.dt = 0.5,
       .soft_limit_active = true,
       .hard_limit_active = false,
       .slew_limit_active = false},
      {.dt = 0.2,
       .soft_limit_active = true,
       .hard_limit_active = true,
       .slew_limit_active = false},
      {.dt = 0.2,
       .soft_limit_active = false,
       .hard_limit_active = true,
       .slew_limit_active = false},
      {.dt = 0.1,
       .soft_limit_active = false,
       .hard_limit_active = false,
       .slew_limit_active = true},
  }};
  const ActiveTimeRatios RATIOS = active_time_ratios(LIMIT_SAMPLES);
  CHECK(RATIOS.valid);
  CHECK_NEAR_DOUBLE(RATIOS.soft_ratio, 0.7, 1.0e-14);
  CHECK_NEAR_DOUBLE(RATIOS.hard_ratio, 0.4, 1.0e-14);
  CHECK_NEAR_DOUBLE(RATIOS.soft_hard_union_ratio, 0.9, 1.0e-14);
  CHECK_NEAR_DOUBLE(RATIOS.slew_ratio, 0.1, 1.0e-14);
}

static double phase_signal(double time) {
  constexpr double FREQUENCY = 1.0;
  constexpr double GAIN = 0.8;
  constexpr double OFFSET = 0.25;
  const double PHASE_LAG = deg_to_rad(15.0);
  return OFFSET +
         GAIN * std::sin(2.0 * std::numbers::pi * FREQUENCY * time - PHASE_LAG);
}

static void test_phase_fit() {
  std::vector<PhaseSample> samples;
  for (int index = 0; index < 8; ++index) {
    const double TIME = static_cast<double>(index) / 8.0;
    samples.push_back({.time = TIME,
                       .value = phase_signal(TIME),
                       .dt = 0.125,
                       .soft_limit_active = false,
                       .hard_limit_active = false});
  }
  samples.push_back({.time = 0.0,
                     .value = 1000.0,
                     .dt = 1.0e-12,
                     .soft_limit_active = false,
                     .hard_limit_active = false});

  const PhaseFit FIT = fit_phase(samples, 1.0, 1.0);
  CHECK(FIT.valid);
  CHECK_NEAR_DOUBLE(FIT.offset, 0.25, 1.0e-8);
  CHECK_NEAR_DOUBLE(FIT.gain, 0.8, 1.0e-8);
  CHECK_NEAR_DOUBLE(rad_to_deg(FIT.phase_lag), 15.0, 1.0e-6);

  constexpr std::array<double, 4> TIMES{{0.0, 0.25, 0.5, 0.75}};
  constexpr std::array<double, 4> DTS{{0.01, 0.33, 0.33, 0.33}};
  std::array<PhaseSample, 4> threshold_samples{};
  for (std::size_t index = 0; index < threshold_samples.size(); ++index) {
    threshold_samples[index] = {
        .time = TIMES[index],
        .value = phase_signal(TIMES[index]),
        .dt = DTS[index],
        .soft_limit_active = index == 0U,
        .hard_limit_active = index == 0U,
    };
  }
  const PhaseFit THRESHOLD_FIT = fit_phase(threshold_samples, 1.0, 1.0);
  CHECK_NEAR_DOUBLE(THRESHOLD_FIT.soft_hard_union_ratio, 0.01, 1.0e-14);
  CHECK(!THRESHOLD_FIT.valid);

  constexpr std::size_t SAMPLE_COUNT = 100U;
  constexpr double SAMPLE_DT = 0.002;
  std::array<PhaseSample, SAMPLE_COUNT> repeated_dt_samples{};
  for (std::size_t index = 0; index < repeated_dt_samples.size(); ++index) {
    const double TIME = static_cast<double>(index) * SAMPLE_DT;
    repeated_dt_samples[index] = {
        .time = TIME,
        .value = phase_signal(TIME),
        .dt = SAMPLE_DT,
        .soft_limit_active = index == 0U,
        .hard_limit_active = false,
    };
  }
  const PhaseFit REPEATED_DT_THRESHOLD_FIT =
      fit_phase(repeated_dt_samples, 1.0, 1.0);
  CHECK_NEAR_DOUBLE(REPEATED_DT_THRESHOLD_FIT.soft_hard_union_ratio, 0.01,
                    1.0e-14);
  CHECK(!REPEATED_DT_THRESHOLD_FIT.valid);
}

int main() {
  test_physical_torque_bounds();
  test_sine_reference_derivatives();
  test_minimum_jerk_boundary_conditions();
  test_general_quintic_boundary_conditions();
  test_exact_zoh_plant();
  test_legacy_adapter_first_tick();
  test_time_weighted_metrics();
  test_phase_fit();
  return yaw_test_failures == 0 ? 0 : 1;
}

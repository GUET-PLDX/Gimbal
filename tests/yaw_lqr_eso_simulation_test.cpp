#include "yaw_lqr_eso_simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "yaw_lqr_eso_test_support.hpp"

enum class ControllerKind {
  LEGACY,
  LQR_1_1,
  LQR_3_8_1_1,
  LQR_12_3_4,
};

enum class ScenarioKind {
  HOLD,
  STEP,
  SINE,
  OVERLOAD,
};

struct Scenario {
  std::string_view name;
  ScenarioKind kind{};
  double amplitude_rad{};
  double frequency_hz{};
  bool feasible_sine{};
};

struct DtMode {
  std::string_view name;
  std::array<double, 4> sequence{};
  std::size_t count{};
};

struct SimulationResult {
  std::string_view scenario;
  bool feasible_sine{};
  std::string_view dt_mode;
  double plant_j{};
  double plant_b{};
  double disturbance{};
  ControllerKind controller{};
  double theta_rmse{};
  double theta_p95{};
  double omega_rmse{};
  double phase_deg{};
  bool phase_valid{};
  double tau_peak{};
  double tau_rms{};
  double soft_ratio{};
  double hard_ratio{};
  double slew_ratio{};
  double max_abs_error{};
  double max_abs_omega{};
  double overshoot_deg{};
  double settling_s{-1.0};
  double recovery_s{-1.0};
  double eso_torque_error_rmse{};
  bool eso_metric_valid{};
  double soft_hard_union_ratio{};
  double final_max_abs_error{};
  double final_max_abs_omega{};
  double step_prior_rmse{};
  double step_final_rmse{};
  double measurement_duration{};
  double expected_measurement_duration{};
  bool final_soft_hard_clear{};
  bool constraints_respected{};
  bool finite{};
};

struct ControllerSample {
  double torque{};
  double z3{};
  bool valid{};
  bool observer_ready{};
  bool soft_limit_active{};
  bool hard_limit_active{};
  bool slew_limit_active{};
};

constexpr std::array<Scenario, 10> SCENARIOS{{
    {.name = "hold", .kind = ScenarioKind::HOLD},
    {.name = "step_pos_5deg",
     .kind = ScenarioKind::STEP,
     .amplitude_rad = deg_to_rad(5.0)},
    {.name = "step_neg_5deg",
     .kind = ScenarioKind::STEP,
     .amplitude_rad = deg_to_rad(-5.0)},
    {.name = "sine_1hz_10deg",
     .kind = ScenarioKind::SINE,
     .amplitude_rad = deg_to_rad(10.0),
     .frequency_hz = 1.0,
     .feasible_sine = true},
    {.name = "sine_3hz_5deg",
     .kind = ScenarioKind::SINE,
     .amplitude_rad = deg_to_rad(5.0),
     .frequency_hz = 3.0,
     .feasible_sine = true},
    {.name = "sine_3hz_8deg",
     .kind = ScenarioKind::SINE,
     .amplitude_rad = deg_to_rad(8.0),
     .frequency_hz = 3.0,
     .feasible_sine = true},
    {.name = "sine_5hz_2deg",
     .kind = ScenarioKind::SINE,
     .amplitude_rad = deg_to_rad(2.0),
     .frequency_hz = 5.0,
     .feasible_sine = true},
    {.name = "sine_5hz_3deg",
     .kind = ScenarioKind::SINE,
     .amplitude_rad = deg_to_rad(3.0),
     .frequency_hz = 5.0,
     .feasible_sine = true},
    {.name = "sine_3hz_10deg",
     .kind = ScenarioKind::SINE,
     .amplitude_rad = deg_to_rad(10.0),
     .frequency_hz = 3.0},
    {.name = "overload_5hz_10deg",
     .kind = ScenarioKind::OVERLOAD,
     .amplitude_rad = deg_to_rad(10.0),
     .frequency_hz = 5.0},
}};

constexpr std::array<ControllerKind, 4> CONTROLLERS{{
    ControllerKind::LEGACY,
    ControllerKind::LQR_1_1,
    ControllerKind::LQR_3_8_1_1,
    ControllerKind::LQR_12_3_4,
}};

constexpr std::size_t NOMINAL_ROWS = 10U * 2U * 4U;
constexpr std::size_t GRID_ROWS = 3U * 4U * 3U * 2U * 2U * 4U;
constexpr std::array<DtMode, 2> DT_MODES{{
    {.name = "fixed", .sequence = {0.002, 0.0, 0.0, 0.0}, .count = 1U},
    {.name = "jitter", .sequence = {0.0015, 0.002, 0.0025, 0.002}, .count = 4U},
}};
constexpr std::array<double, 3> PLANT_J_VALUES{{0.021, 0.030, 0.039}};
constexpr std::array<double, 4> PLANT_B_VALUES{{0.0, 0.1, 0.2, 0.3}};
constexpr std::array<double, 3> DISTURBANCE_VALUES{{-0.2, 0.0, 0.2}};
constexpr std::array<std::size_t, 2> MISMATCH_SCENARIO_INDICES{{5U, 7U}};

class ControllerAdapter final {
 public:
  explicit ControllerAdapter(ControllerKind kind)
      : kind_(kind), config_(base_yaw_config()) {
    switch (kind_) {
      case ControllerKind::LEGACY:
      case ControllerKind::LQR_1_1:
        break;
      case ControllerKind::LQR_3_8_1_1:
        config_.k_theta = 3.8f;
        config_.k_omega = 1.1f;
        break;
      case ControllerKind::LQR_12_3_4:
        config_.k_theta = 12.0f;
        config_.k_omega = 3.4f;
        break;
    }
  }

  void Reset(PlantState state) {
    legacy_.Reset();
    controller_.Reset(static_cast<float>(wrap_pi(state.theta)),
                      static_cast<float>(state.omega), 0.0f);
  }

  ControllerSample Calculate(ReferenceSample reference, PlantState feedback,
                             double dt) {
    if (kind_ == ControllerKind::LEGACY) {
      const double TORQUE =
          legacy_.Calculate(reference, feedback.theta, feedback.omega, dt);
      constexpr double LEGACY_TORQUE_LIMIT = 2.223;
      return {
          .torque = TORQUE,
          .valid = std::isfinite(TORQUE),
          .hard_limit_active = std::fabs(TORQUE) >= LEGACY_TORQUE_LIMIT,
      };
    }

    const YawLqrEso::Output OUTPUT = controller_.Calculate(
        config_,
        {.theta_rad = static_cast<float>(reference.theta),
         .omega_rad_s = static_cast<float>(reference.omega),
         .alpha_rad_s2 = static_cast<float>(reference.alpha)},
        {.theta_rad = static_cast<float>(feedback.theta),
         .omega_rad_s = static_cast<float>(feedback.omega),
         .tau_meas_nm = 0.0f,
         .valid = true,
         .torque_measurement_valid = true},
        static_cast<float>(dt), TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM);
    return {
        .torque = static_cast<double>(OUTPUT.tau_cmd_nm),
        .z3 = static_cast<double>(OUTPUT.z3),
        .valid = OUTPUT.valid,
        .observer_ready = OUTPUT.observer_ready,
        .soft_limit_active = OUTPUT.soft_limit_active,
        .hard_limit_active = OUTPUT.hard_limit_active,
        .slew_limit_active = OUTPUT.slew_limit_active,
    };
  }

  void CommitAppliedTorque(double torque) {
    if (kind_ != ControllerKind::LEGACY) {
      controller_.CommitAppliedTorque(static_cast<float>(torque));
    }
  }

  const YawLqrEso::Config& Config() const { return config_; }

  bool IsLegacy() const { return kind_ == ControllerKind::LEGACY; }

 private:
  ControllerKind kind_;
  YawLqrEso::Config config_;
  LegacyYawAdapter legacy_;
  YawLqrEso controller_;
};

struct TraceSample {
  double time{};
  double dt{};
  double error{};
  double omega{};
  bool soft_limit_active{};
  bool hard_limit_active{};
};

static ReferenceSample scenario_reference(const Scenario& scenario,
                                          double time) {
  constexpr double STEP_RISE_TIME = 0.2;
  constexpr double OVERLOAD_PERIOD_COUNT = 3.25;
  constexpr double OVERLOAD_RETURN_TIME = 0.2;
  switch (scenario.kind) {
    case ScenarioKind::HOLD:
      return {};
    case ScenarioKind::STEP:
      if (time < STEP_RISE_TIME) {
        return minimum_jerk_reference(0.0, scenario.amplitude_rad,
                                      STEP_RISE_TIME, time);
      }
      return {.theta = scenario.amplitude_rad};
    case ScenarioKind::SINE:
      return sine_reference(scenario.amplitude_rad, scenario.frequency_hz,
                            time);
    case ScenarioKind::OVERLOAD: {
      const double SWITCH_TIME = OVERLOAD_PERIOD_COUNT / scenario.frequency_hz;
      if (time <= SWITCH_TIME) {
        return sine_reference(scenario.amplitude_rad, scenario.frequency_hz,
                              time);
      }
      const ReferenceSample SWITCH_REFERENCE = sine_reference(
          scenario.amplitude_rad, scenario.frequency_hz, SWITCH_TIME);
      constexpr ReferenceSample FINISH_REFERENCE{};
      const QuinticTrajectory RETURN_TRAJECTORY(
          SWITCH_REFERENCE, FINISH_REFERENCE, OVERLOAD_RETURN_TIME);
      if (time < SWITCH_TIME + OVERLOAD_RETURN_TIME) {
        return RETURN_TRAJECTORY.Sample(time - SWITCH_TIME);
      }
      return FINISH_REFERENCE;
    }
  }
  return {};
}

static double scenario_duration(const Scenario& scenario) {
  constexpr double HOLD_TIME = 5.0;
  constexpr double STEP_RISE_TIME = 0.2;
  constexpr double STEP_HOLD_TIME = 5.0;
  constexpr double SINE_PERIOD_COUNT = 12.0;
  constexpr double OVERLOAD_PERIOD_COUNT = 3.25;
  constexpr double OVERLOAD_RETURN_TIME = 0.2;
  constexpr double OVERLOAD_HOLD_TIME = 5.0;
  switch (scenario.kind) {
    case ScenarioKind::HOLD:
      return HOLD_TIME;
    case ScenarioKind::STEP:
      return STEP_RISE_TIME + STEP_HOLD_TIME;
    case ScenarioKind::SINE:
      return SINE_PERIOD_COUNT / scenario.frequency_hz;
    case ScenarioKind::OVERLOAD:
      return OVERLOAD_PERIOD_COUNT / scenario.frequency_hz +
             OVERLOAD_RETURN_TIME + OVERLOAD_HOLD_TIME;
  }
  return 0.0;
}

static double measurement_start(const Scenario& scenario) {
  constexpr double WARMUP_PERIOD_COUNT = 2.0;
  if (scenario.kind == ScenarioKind::SINE) {
    return WARMUP_PERIOD_COUNT / scenario.frequency_hz;
  }
  return 0.0;
}

static double recovery_start(const Scenario& scenario) {
  constexpr double OVERLOAD_PERIOD_COUNT = 3.25;
  constexpr double OVERLOAD_RETURN_TIME = 0.2;
  return OVERLOAD_PERIOD_COUNT / scenario.frequency_hz + OVERLOAD_RETURN_TIME;
}

static double interval_overlap(double sample_start, double sample_dt,
                               double window_start, double window_finish) {
  const double OVERLAP_START = std::max(sample_start, window_start);
  const double OVERLAP_FINISH =
      std::min(sample_start + sample_dt, window_finish);
  return std::max(0.0, OVERLAP_FINISH - OVERLAP_START);
}

static double trace_rmse(const std::vector<TraceSample>& samples, double start,
                         double finish) {
  std::vector<WeightedValueSample> values;
  for (const TraceSample& SAMPLE : samples) {
    const double WEIGHT =
        interval_overlap(SAMPLE.time, SAMPLE.dt, start, finish);
    if (WEIGHT > 0.0) {
      values.push_back({.value = SAMPLE.error, .dt = WEIGHT});
    }
  }
  return weighted_rmse(values);
}

static double calculate_settling_time(const std::vector<TraceSample>& samples,
                                      double duration) {
  constexpr double STEP_RISE_TIME = 0.2;
  constexpr double SETTLING_ERROR_LIMIT = deg_to_rad(1.0);
  double settled_start = STEP_RISE_TIME;
  bool saw_hold_sample = false;
  for (const TraceSample& SAMPLE : samples) {
    const double HOLD_DT =
        interval_overlap(SAMPLE.time, SAMPLE.dt, STEP_RISE_TIME, duration);
    if (HOLD_DT <= 0.0) {
      continue;
    }
    saw_hold_sample = true;
    if (std::fabs(SAMPLE.error) > SETTLING_ERROR_LIMIT) {
      settled_start = std::min(SAMPLE.time + SAMPLE.dt, duration);
    }
  }
  if (!saw_hold_sample || settled_start > duration) {
    return -1.0;
  }
  return std::max(0.0, settled_start - STEP_RISE_TIME);
}

static double calculate_recovery_time(const std::vector<TraceSample>& samples,
                                      double start, double finish) {
  constexpr double RECOVERY_WINDOW = 1.0;
  constexpr double RECOVERY_ERROR_LIMIT = deg_to_rad(1.0);
  constexpr double RECOVERY_OMEGA_LIMIT = 0.2;
  double valid_start = -1.0;
  double valid_duration = 0.0;
  for (const TraceSample& SAMPLE : samples) {
    const double RECOVERY_DT =
        interval_overlap(SAMPLE.time, SAMPLE.dt, start, finish);
    if (RECOVERY_DT <= 0.0) {
      continue;
    }
    const bool VALID = std::fabs(SAMPLE.error) <= RECOVERY_ERROR_LIMIT &&
                       std::fabs(SAMPLE.omega) <= RECOVERY_OMEGA_LIMIT &&
                       !SAMPLE.soft_limit_active && !SAMPLE.hard_limit_active;
    if (!VALID) {
      valid_start = -1.0;
      valid_duration = 0.0;
      continue;
    }
    if (valid_start < 0.0) {
      valid_start = std::max(SAMPLE.time, start);
    }
    valid_duration += RECOVERY_DT;
    if (valid_duration >= RECOVERY_WINDOW) {
      return std::max(0.0, valid_start - start);
    }
  }
  return -1.0;
}

static bool result_numbers_are_finite(const SimulationResult& result) {
  return std::isfinite(result.plant_j) && std::isfinite(result.plant_b) &&
         std::isfinite(result.disturbance) &&
         std::isfinite(result.theta_rmse) && std::isfinite(result.theta_p95) &&
         std::isfinite(result.omega_rmse) && std::isfinite(result.phase_deg) &&
         std::isfinite(result.tau_peak) && std::isfinite(result.tau_rms) &&
         std::isfinite(result.soft_ratio) && std::isfinite(result.hard_ratio) &&
         std::isfinite(result.slew_ratio) &&
         std::isfinite(result.max_abs_error) &&
         std::isfinite(result.max_abs_omega) &&
         std::isfinite(result.overshoot_deg) &&
         std::isfinite(result.settling_s) && std::isfinite(result.recovery_s) &&
         std::isfinite(result.eso_torque_error_rmse) &&
         std::isfinite(result.soft_hard_union_ratio) &&
         std::isfinite(result.final_max_abs_error) &&
         std::isfinite(result.final_max_abs_omega) &&
         std::isfinite(result.step_prior_rmse) &&
         std::isfinite(result.step_final_rmse) &&
         std::isfinite(result.measurement_duration) &&
         std::isfinite(result.expected_measurement_duration);
}

static SimulationResult run_simulation_case(const Scenario& scenario,
                                            const DtMode& dt_mode,
                                            double plant_j, double plant_b,
                                            double disturbance,
                                            ControllerKind controller_kind) {
  constexpr double HARD_CONSTRAINT_TOLERANCE = 1.0e-5;
  constexpr double SLEW_CONSTRAINT_TOLERANCE = 1.0e-5;
  constexpr double STEP_RISE_TIME = 0.2;
  const double DURATION = scenario_duration(scenario);
  const double MEASUREMENT_START = measurement_start(scenario);
  const ReferenceSample INITIAL_REFERENCE = scenario_reference(scenario, 0.0);
  PlantState state{.theta = INITIAL_REFERENCE.theta,
                   .omega = INITIAL_REFERENCE.omega};
  ControllerAdapter adapter(controller_kind);
  adapter.Reset(state);

  std::vector<WeightedValueSample> theta_error_samples;
  std::vector<WeightedValueSample> omega_error_samples;
  std::vector<WeightedValueSample> torque_samples;
  std::vector<WeightedValueSample> eso_error_samples;
  std::vector<LimitActivitySample> limit_samples;
  std::vector<PhaseSample> phase_samples;
  std::vector<TraceSample> trace_samples;
  const std::size_t RESERVE_COUNT =
      static_cast<std::size_t>(std::ceil(DURATION / 0.0015)) + 1U;
  theta_error_samples.reserve(RESERVE_COUNT);
  omega_error_samples.reserve(RESERVE_COUNT);
  torque_samples.reserve(RESERVE_COUNT);
  eso_error_samples.reserve(RESERVE_COUNT);
  limit_samples.reserve(RESERVE_COUNT);
  phase_samples.reserve(RESERVE_COUNT);
  trace_samples.reserve(RESERVE_COUNT);

  bool finite = true;
  bool constraints_respected = true;
  double previous_torque = 0.0;
  double max_abs_error = 0.0;
  double max_abs_omega = 0.0;
  double tau_peak = 0.0;
  double overshoot = 0.0;
  double recorded_duration = 0.0;
  double time = 0.0;
  std::size_t step = 0U;
  while (time < DURATION) {
    const double DT = dt_mode.sequence[step % dt_mode.count];
    const ReferenceSample REFERENCE = scenario_reference(scenario, time);
    const PlantState WRAPPED_FEEDBACK{.theta = wrap_pi(state.theta),
                                      .omega = state.omega};
    const ControllerSample OUTPUT =
        adapter.Calculate(REFERENCE, WRAPPED_FEEDBACK, DT);
    adapter.CommitAppliedTorque(OUTPUT.torque);

    const double ERROR = wrap_pi(WRAPPED_FEEDBACK.theta - REFERENCE.theta);
    const double OMEGA_ERROR = WRAPPED_FEEDBACK.omega - REFERENCE.omega;
    const double MEASUREMENT_DT =
        interval_overlap(time, DT, MEASUREMENT_START, DURATION);
    finite = finite && OUTPUT.valid && std::isfinite(OUTPUT.torque) &&
             std::isfinite(ERROR) && std::isfinite(OMEGA_ERROR) &&
             std::isfinite(state.theta) && std::isfinite(state.omega);
    max_abs_error = std::max(max_abs_error, std::fabs(ERROR));
    max_abs_omega = std::max(max_abs_omega, std::fabs(state.omega));

    const YawLqrEso::Config& CONFIG = adapter.Config();
    constraints_respected = constraints_respected &&
                            std::fabs(OUTPUT.torque) <=
                                static_cast<double>(TEST_YAW_TORQUE_LIMIT_NM) +
                                    HARD_CONSTRAINT_TOLERANCE;
    if (!adapter.IsLegacy()) {
      if (CONFIG.torque_slew_enable) {
        const double MAXIMUM_DELTA =
            static_cast<double>(CONFIG.torque_slew_rate_nm_s) * DT +
            SLEW_CONSTRAINT_TOLERANCE;
        constraints_respected =
            constraints_respected &&
            std::fabs(OUTPUT.torque - previous_torque) <= MAXIMUM_DELTA;
      }
    }
    previous_torque = OUTPUT.torque;

    trace_samples.push_back({.time = time,
                             .dt = DT,
                             .error = ERROR,
                             .omega = state.omega,
                             .soft_limit_active = OUTPUT.soft_limit_active,
                             .hard_limit_active = OUTPUT.hard_limit_active});
    if (scenario.kind == ScenarioKind::STEP && time >= STEP_RISE_TIME) {
      const double DIRECTION = scenario.amplitude_rad >= 0.0 ? 1.0 : -1.0;
      overshoot = std::max(overshoot,
                           DIRECTION * (state.theta - scenario.amplitude_rad));
    }
    if (MEASUREMENT_DT > 0.0) {
      recorded_duration += MEASUREMENT_DT;
      theta_error_samples.push_back({.value = ERROR, .dt = MEASUREMENT_DT});
      omega_error_samples.push_back(
          {.value = OMEGA_ERROR, .dt = MEASUREMENT_DT});
      torque_samples.push_back({.value = OUTPUT.torque, .dt = MEASUREMENT_DT});
      limit_samples.push_back({.dt = MEASUREMENT_DT,
                               .soft_limit_active = OUTPUT.soft_limit_active,
                               .hard_limit_active = OUTPUT.hard_limit_active,
                               .slew_limit_active = OUTPUT.slew_limit_active});
      tau_peak = std::max(tau_peak, std::fabs(OUTPUT.torque));
      if (scenario.kind == ScenarioKind::SINE) {
        phase_samples.push_back(
            {.time = std::max(time, MEASUREMENT_START),
             .value = state.theta,
             .dt = MEASUREMENT_DT,
             .soft_limit_active = OUTPUT.soft_limit_active,
             .hard_limit_active = OUTPUT.hard_limit_active});
      }
      if (!adapter.IsLegacy() && OUTPUT.observer_ready) {
        const double ESTIMATED_DISTURBANCE_TORQUE =
            static_cast<double>(TEST_YAW_J_KG_M2) * OUTPUT.z3;
        eso_error_samples.push_back(
            {.value = ESTIMATED_DISTURBANCE_TORQUE - disturbance,
             .dt = MEASUREMENT_DT});
      }
    }

    state = propagate_exact_zoh(state, plant_j, plant_b, OUTPUT.torque,
                                disturbance, DT);
    time += DT;
    ++step;
  }

  SimulationResult result{
      .scenario = scenario.name,
      .feasible_sine = scenario.feasible_sine,
      .dt_mode = dt_mode.name,
      .plant_j = plant_j,
      .plant_b = plant_b,
      .disturbance = disturbance,
      .controller = controller_kind,
      .theta_rmse = weighted_rmse(theta_error_samples),
      .theta_p95 = weighted_absolute_p95(theta_error_samples),
      .omega_rmse = weighted_rmse(omega_error_samples),
      .tau_peak = tau_peak,
      .tau_rms = weighted_rmse(torque_samples),
      .max_abs_error = max_abs_error,
      .max_abs_omega = max_abs_omega,
      .overshoot_deg = rad_to_deg(std::max(0.0, overshoot)),
      .measurement_duration = recorded_duration,
      .expected_measurement_duration = DURATION - MEASUREMENT_START,
      .constraints_respected = constraints_respected,
  };
  const ActiveTimeRatios RATIOS = active_time_ratios(limit_samples);
  result.soft_ratio = RATIOS.soft_ratio;
  result.hard_ratio = RATIOS.hard_ratio;
  result.slew_ratio = RATIOS.slew_ratio;
  result.soft_hard_union_ratio = RATIOS.soft_hard_union_ratio;

  if (scenario.kind == ScenarioKind::SINE) {
    const PhaseFit PHASE = fit_phase(phase_samples, scenario.frequency_hz,
                                     std::fabs(scenario.amplitude_rad));
    result.phase_valid = PHASE.valid;
    result.phase_deg = PHASE.valid ? rad_to_deg(PHASE.phase_lag) : 0.0;
  }
  if (!eso_error_samples.empty()) {
    result.eso_metric_valid = true;
    result.eso_torque_error_rmse = weighted_rmse(eso_error_samples);
  }

  const double FINAL_WINDOW_START = DURATION - 1.0;
  result.final_soft_hard_clear = true;
  bool saw_final_sample = false;
  for (const TraceSample& SAMPLE : trace_samples) {
    const double FINAL_DT =
        interval_overlap(SAMPLE.time, SAMPLE.dt, FINAL_WINDOW_START, DURATION);
    if (FINAL_DT <= 0.0) {
      continue;
    }
    saw_final_sample = true;
    result.final_max_abs_error =
        std::max(result.final_max_abs_error, std::fabs(SAMPLE.error));
    result.final_max_abs_omega =
        std::max(result.final_max_abs_omega, std::fabs(SAMPLE.omega));
    result.final_soft_hard_clear = result.final_soft_hard_clear &&
                                   !SAMPLE.soft_limit_active &&
                                   !SAMPLE.hard_limit_active;
  }
  result.final_soft_hard_clear =
      result.final_soft_hard_clear && saw_final_sample;

  if (scenario.kind == ScenarioKind::STEP) {
    result.settling_s = calculate_settling_time(trace_samples, DURATION);
    result.step_prior_rmse =
        trace_rmse(trace_samples, DURATION - 2.0, DURATION - 1.0);
    result.step_final_rmse =
        trace_rmse(trace_samples, DURATION - 1.0, DURATION);
  }
  if (scenario.kind == ScenarioKind::OVERLOAD) {
    result.recovery_s = calculate_recovery_time(
        trace_samples, recovery_start(scenario), DURATION);
  }

  finite = finite && std::isfinite(state.theta) && std::isfinite(state.omega) &&
           RATIOS.valid && !theta_error_samples.empty() &&
           !omega_error_samples.empty() && !torque_samples.empty();
  result.finite = finite && result_numbers_are_finite(result);
  return result;
}

static SimulationResult run_smoke_case(ControllerKind kind) {
  constexpr double AMPLITUDE = deg_to_rad(10.0);
  constexpr double FREQUENCY = 1.0;
  constexpr double DT = 0.002;
  constexpr double DURATION = 12.0;
  const std::size_t STEP_COUNT =
      static_cast<std::size_t>(std::ceil(DURATION / DT));

  const ReferenceSample INITIAL_REFERENCE =
      sine_reference(AMPLITUDE, FREQUENCY, 0.0);
  PlantState state{.theta = INITIAL_REFERENCE.theta,
                   .omega = INITIAL_REFERENCE.omega};
  ControllerAdapter adapter(kind);
  adapter.Reset(state);
  bool finite = true;
  double time = 0.0;
  for (std::size_t step = 0; step < STEP_COUNT; ++step) {
    const ReferenceSample REFERENCE =
        sine_reference(AMPLITUDE, FREQUENCY, time);
    const PlantState WRAPPED_FEEDBACK{.theta = wrap_pi(state.theta),
                                      .omega = state.omega};
    const ControllerSample OUTPUT =
        adapter.Calculate(REFERENCE, WRAPPED_FEEDBACK, DT);
    adapter.CommitAppliedTorque(OUTPUT.torque);
    finite = finite && OUTPUT.valid && std::isfinite(OUTPUT.torque) &&
             std::isfinite(state.theta) && std::isfinite(state.omega);
    state = propagate_exact_zoh(state, 0.03, 0.0, OUTPUT.torque, 0.0, DT);
    time += DT;
  }
  finite = finite && std::isfinite(state.theta) && std::isfinite(state.omega);
  SimulationResult result{};
  result.finite = finite;
  return result;
}

static std::vector<SimulationResult> run_smoke_cases() {
  std::vector<SimulationResult> results;
  results.reserve(CONTROLLERS.size());
  for (const ControllerKind KIND : CONTROLLERS) {
    results.push_back(run_smoke_case(KIND));
  }
  return results;
}

static std::vector<SimulationResult> run_nominal_matrix() {
  constexpr double NOMINAL_PLANT_J = 0.03;
  constexpr double NOMINAL_PLANT_B = 0.0;
  constexpr double NOMINAL_DISTURBANCE = 0.0;
  std::vector<SimulationResult> results;
  results.reserve(NOMINAL_ROWS);
  for (const Scenario& SCENARIO : SCENARIOS) {
    for (const DtMode& DT_MODE : DT_MODES) {
      for (const ControllerKind CONTROLLER : CONTROLLERS) {
        results.push_back(run_simulation_case(SCENARIO, DT_MODE,
                                              NOMINAL_PLANT_J, NOMINAL_PLANT_B,
                                              NOMINAL_DISTURBANCE, CONTROLLER));
      }
    }
  }
  return results;
}

static std::vector<SimulationResult> run_mismatch_matrix() {
  std::vector<SimulationResult> results;
  results.reserve(GRID_ROWS);
  for (const double PLANT_J : PLANT_J_VALUES) {
    for (const double PLANT_B : PLANT_B_VALUES) {
      for (const double DISTURBANCE : DISTURBANCE_VALUES) {
        for (const std::size_t SCENARIO_INDEX : MISMATCH_SCENARIO_INDICES) {
          const Scenario& SCENARIO = SCENARIOS[SCENARIO_INDEX];
          for (const DtMode& DT_MODE : DT_MODES) {
            for (const ControllerKind CONTROLLER : CONTROLLERS) {
              results.push_back(run_simulation_case(SCENARIO, DT_MODE, PLANT_J,
                                                    PLANT_B, DISTURBANCE,
                                                    CONTROLLER));
            }
          }
        }
      }
    }
  }
  return results;
}

static std::string_view controller_name(ControllerKind kind) {
  switch (kind) {
    case ControllerKind::LEGACY:
      return "legacy";
    case ControllerKind::LQR_1_1:
      return "[1,1]";
    case ControllerKind::LQR_3_8_1_1:
      return "[3.8,1.1]";
    case ControllerKind::LQR_12_3_4:
      return "[12,3.4]";
  }
  return "unknown";
}

static void check_gate(bool ok, const SimulationResult& result,
                       const char* gate, int line) {
  if (ok) {
    return;
  }
  const std::string_view CONTROLLER = controller_name(result.controller);
  std::fprintf(stderr,
               "FAIL line %d gate=%s scenario=%.*s dt=%.*s J=%.17g B=%.17g "
               "d=%.17g controller=%.*s rmse=%.17g union=%.17g max_e=%.17g "
               "max_w=%.17g final_e=%.17g final_w=%.17g recovery=%.17g\n",
               line, gate, static_cast<int>(result.scenario.size()),
               result.scenario.data(), static_cast<int>(result.dt_mode.size()),
               result.dt_mode.data(), result.plant_j, result.plant_b,
               result.disturbance, static_cast<int>(CONTROLLER.size()),
               CONTROLLER.data(), result.theta_rmse,
               result.soft_hard_union_ratio, result.max_abs_error,
               result.max_abs_omega, result.final_max_abs_error,
               result.final_max_abs_omega, result.recovery_s);
  ++yaw_test_failures;
}

#define CHECK_GATE(CONDITION, RESULT, GATE) \
  check_gate((CONDITION), (RESULT), (GATE), __LINE__)

static bool is_nominal_plant(const SimulationResult& result) {
  return result.plant_j == 0.03 && result.plant_b == 0.0 &&
         result.disturbance == 0.0;
}

static bool is_step_scenario(std::string_view name) {
  return name == "step_pos_5deg" || name == "step_neg_5deg";
}

static void test_non_performance_gates(
    const std::vector<SimulationResult>& results) {
  constexpr double ERROR_LIMIT = deg_to_rad(1.0);
  constexpr double OMEGA_LIMIT = 0.2;
  constexpr double RMSE_COMPARISON_TOLERANCE = 1.0e-12;
  constexpr double DURATION_TOLERANCE = 1.0e-12;
  for (const SimulationResult& RESULT : results) {
    CHECK_GATE(RESULT.finite, RESULT, "finite_complete_result");
    CHECK_GATE(RESULT.constraints_respected, RESULT,
               "hard_and_slew_constraints");
    CHECK_GATE(
        std::fabs(RESULT.measurement_duration -
                  RESULT.expected_measurement_duration) <= DURATION_TOLERANCE,
        RESULT, "exact_measurement_duration");
    if (RESULT.controller == ControllerKind::LEGACY) {
      CHECK_GATE(!RESULT.eso_metric_valid, RESULT, "legacy_eso_metric_invalid");
    }

    if (RESULT.controller != ControllerKind::LQR_1_1) {
      continue;
    }
    CHECK_GATE(RESULT.max_abs_error < std::numbers::pi, RESULT,
               "bounded_cyclic_error");
    CHECK_GATE(RESULT.max_abs_omega < 100.0, RESULT, "bounded_omega");

    if (!is_nominal_plant(RESULT)) {
      continue;
    }
    if (is_step_scenario(RESULT.scenario)) {
      CHECK_GATE(RESULT.final_max_abs_error <= ERROR_LIMIT, RESULT,
                 "step_final_error");
      CHECK_GATE(RESULT.step_final_rmse <=
                     RESULT.step_prior_rmse + RMSE_COMPARISON_TOLERANCE,
                 RESULT, "step_nonincreasing_rmse");
    }
    if (RESULT.feasible_sine) {
      CHECK_GATE(RESULT.soft_hard_union_ratio < 0.5, RESULT,
                 "feasible_sine_limit_ratio");
    }
    if (RESULT.scenario == "overload_5hz_10deg") {
      CHECK_GATE(RESULT.soft_hard_union_ratio > 0.0, RESULT,
                 "overload_reaches_limit");
      CHECK_GATE(!RESULT.phase_valid, RESULT, "overload_phase_invalid");
      CHECK_GATE(RESULT.final_max_abs_error <= ERROR_LIMIT, RESULT,
                 "overload_final_error");
      CHECK_GATE(RESULT.final_max_abs_omega <= OMEGA_LIMIT, RESULT,
                 "overload_final_omega");
      CHECK_GATE(RESULT.final_soft_hard_clear, RESULT,
                 "overload_final_limits_clear");
      CHECK_GATE(RESULT.recovery_s >= 0.0, RESULT, "overload_recovery_found");
    }
  }
}

static bool write_csv(const std::vector<SimulationResult>& results) {
  const char* BUILD_DIRECTORY = std::getenv("YAW_TEST_BUILD_DIR");
  if (BUILD_DIRECTORY == nullptr || BUILD_DIRECTORY[0] == '\0') {
    return false;
  }
  const std::string REPORT_PATH =
      std::string(BUILD_DIRECTORY) + "/yaw_lqr_eso_report.csv";
  std::ofstream report(REPORT_PATH, std::ios::out | std::ios::trunc);
  if (!report.is_open()) {
    return false;
  }
  report << "scenario,dt_mode,plant_j,plant_b,disturbance,controller,"
            "theta_rmse,theta_p95,omega_rmse,phase_deg,phase_valid,tau_peak,"
            "tau_rms,soft_ratio,hard_ratio,slew_ratio,max_abs_error,"
            "max_abs_omega,overshoot_deg,settling_s,recovery_s,"
            "eso_torque_error_rmse,eso_metric_valid\n";
  report << std::fixed << std::setprecision(12);
  for (const SimulationResult& RESULT : results) {
    const std::string_view CONTROLLER = controller_name(RESULT.controller);
    report << RESULT.scenario << ',' << RESULT.dt_mode << ',' << RESULT.plant_j
           << ',' << RESULT.plant_b << ',' << RESULT.disturbance << ",\""
           << CONTROLLER << "\"," << RESULT.theta_rmse << ','
           << RESULT.theta_p95 << ',' << RESULT.omega_rmse << ','
           << RESULT.phase_deg << ',' << static_cast<int>(RESULT.phase_valid)
           << ',' << RESULT.tau_peak << ',' << RESULT.tau_rms << ','
           << RESULT.soft_ratio << ',' << RESULT.hard_ratio << ','
           << RESULT.slew_ratio << ',' << RESULT.max_abs_error << ','
           << RESULT.max_abs_omega << ',' << RESULT.overshoot_deg << ','
           << RESULT.settling_s << ',' << RESULT.recovery_s << ','
           << RESULT.eso_torque_error_rmse << ','
           << static_cast<int>(RESULT.eso_metric_valid) << '\n';
  }
  report.close();
  return report.good();
}

static void test_exact_matrix_counts_and_smoke_cases() {
  const std::vector<SimulationResult> SMOKE_RESULTS = run_smoke_cases();
  CHECK(SMOKE_RESULTS.size() == CONTROLLERS.size());
  for (const SimulationResult& RESULT : SMOKE_RESULTS) {
    CHECK(RESULT.finite);
  }

  std::vector<SimulationResult> nominal_results = run_nominal_matrix();
  const std::vector<SimulationResult> MISMATCH_RESULTS = run_mismatch_matrix();
  CHECK(SCENARIOS.size() == 10U);
  CHECK(NOMINAL_ROWS == 80U);
  CHECK(GRID_ROWS == 576U);
  CHECK(nominal_results.size() == NOMINAL_ROWS);
  for (const SimulationResult& RESULT : nominal_results) {
    CHECK(RESULT.finite);
    CHECK(RESULT.constraints_respected);
  }
  CHECK(MISMATCH_RESULTS.size() == GRID_ROWS);
  for (const SimulationResult& RESULT : MISMATCH_RESULTS) {
    CHECK(RESULT.finite);
    CHECK(RESULT.constraints_respected);
  }
  std::vector<SimulationResult> results = std::move(nominal_results);
  results.reserve(NOMINAL_ROWS + GRID_ROWS);
  results.insert(results.end(), MISMATCH_RESULTS.begin(),
                 MISMATCH_RESULTS.end());
  CHECK(results.size() == NOMINAL_ROWS + GRID_ROWS);
  test_non_performance_gates(results);
  CHECK(write_csv(results));
}

int main() {
  test_exact_matrix_counts_and_smoke_cases();
  return yaw_test_failures == 0 ? 0 : 1;
}

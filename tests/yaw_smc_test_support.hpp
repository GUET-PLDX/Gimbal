#pragma once

#include <cmath>
#include <cstdio>

#include "YawSmc.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

inline int yaw_smc_test_failures = 0;
inline void check(bool ok, const char* expr, int line) {
  if (!ok) {
    std::fprintf(stderr, "FAIL line %d: %s\n", line, expr);
    ++yaw_smc_test_failures;
  }
}
inline void check_near(float actual, float expected, float tolerance,
                       const char* expr, int line) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::fprintf(stderr, "FAIL line %d: %s actual=%g expected=%g\n", line, expr,
                 static_cast<double>(actual), static_cast<double>(expected));
    ++yaw_smc_test_failures;
  }
}
#define CHECK(EXPR) check((EXPR), #EXPR, __LINE__)
#define CHECK_NEAR(ACTUAL, EXPECTED, TOL) \
  check_near((ACTUAL), (EXPECTED), (TOL), #ACTUAL, __LINE__)

inline YawSmc::Config base_yaw_smc_config() {
  return {.j_kg_m2 = 0.03f,
          .c = 20.0f,
          .k = 2.0f,
          .epsilon = 0.5f,
          .q = 21.0f,
          .p = 27.0f,
          .error_deadband_rad = 0.0f,
          .ftsmc_switch_rad = 0.0174533f,
          .sat_boundary = 1.0f,
          .torque_soft_limit_nm = 2.0f,
          .torque_min_nm = -2.223f,
          .torque_max_nm = 2.223f,
          .torque_slew_rate_nm_s = 1000.0f,
          .ftsmc_enable = true,
          .torque_slew_enable = true};
}

inline float sat(float y) {
  if (y > 1.0f) {
    return 1.0f;
  }
  if (y < -1.0f) {
    return -1.0f;
  }
  return y;
}

inline float sig_pow(float value, float exponent) {
  return std::copysign(std::pow(std::fabs(value), exponent), value);
}

// Independent oracle copied from SMC_Tick. It deliberately does not call
// YawSmc helpers so the host test can catch a shared implementation mistake.
struct SmcCodeOracle {
  float e_theta_rad{};
  float e_omega_rad_s{};
  float s{};
  float sat_s{};
  float tau_ff_alpha_nm{};
  float tau_smc_nm{};
  float tau_pre_limit_nm{};
  bool used_ftsmc{};
  bool in_deadband{};
};

inline SmcCodeOracle smc_code_oracle(const YawSmc::Config& config,
                                     const YawSmc::Reference& reference,
                                     const YawSmc::Feedback& feedback,
                                     float target_last_rad = 0.0f,
                                     float target_dot_rad_s = 0.0f) {
  SmcCodeOracle oracle{};
  oracle.e_theta_rad =
      LibXR::CycleValue<float>(feedback.theta_rad) - reference.theta_rad;
  const float TARGET_DELTA_RAD =
      LibXR::CycleValue<float>(reference.theta_rad) - target_last_rad;
  oracle.e_omega_rad_s = feedback.omega_rad_s - target_dot_rad_s;
  oracle.in_deadband =
      std::fabs(oracle.e_theta_rad) < config.error_deadband_rad;
  if (oracle.in_deadband) {
    return oracle;
  }

  oracle.tau_ff_alpha_nm =
      config.j_kg_m2 * (TARGET_DELTA_RAD - target_dot_rad_s);
  const float ABS_E_THETA_RAD = std::fabs(oracle.e_theta_rad);
  oracle.used_ftsmc =
      config.ftsmc_enable && ABS_E_THETA_RAD >= config.ftsmc_switch_rad;
  float surface_dot_term = 0.0f;
  if (oracle.used_ftsmc) {
    const float R = config.q / config.p;
    oracle.s = oracle.e_omega_rad_s + config.c * sig_pow(oracle.e_theta_rad, R);
    const float E_QP = sig_pow(oracle.e_theta_rad, R);
    surface_dot_term =
        config.c * R * oracle.e_omega_rad_s * E_QP / ABS_E_THETA_RAD;
  } else {
    oracle.s = oracle.e_omega_rad_s + config.c * oracle.e_theta_rad;
    surface_dot_term = config.c * oracle.e_omega_rad_s;
  }
  oracle.sat_s = sat(oracle.s / config.sat_boundary);
  oracle.tau_smc_nm =
      config.j_kg_m2 *
      (-surface_dot_term - config.epsilon * oracle.sat_s - config.k * oracle.s);
  oracle.tau_pre_limit_nm = oracle.tau_ff_alpha_nm + oracle.tau_smc_nm;
  return oracle;
}

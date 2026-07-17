#pragma once

#include <cmath>
#include <cstdio>

#include "YawLqrEso.hpp"

inline int yaw_test_failures = 0;
inline void check(bool ok, const char* expr, int line) {
  if (!ok) {
    std::fprintf(stderr, "FAIL line %d: %s\n", line, expr);
    ++yaw_test_failures;
  }
}
inline void check_near(float actual, float expected, float tolerance,
                       const char* expr, int line) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::fprintf(stderr, "FAIL line %d: %s actual=%g expected=%g\n", line, expr,
                 static_cast<double>(actual), static_cast<double>(expected));
    ++yaw_test_failures;
  }
}
#define CHECK(EXPR) check((EXPR), #EXPR, __LINE__)
#define CHECK_NEAR(ACTUAL, EXPECTED, TOL) \
  check_near((ACTUAL), (EXPECTED), (TOL), #ACTUAL, __LINE__)

inline YawLqrEso::Config base_yaw_config() {
  return {.j_kg_m2 = 0.03f,
          .b_nms_rad = 0.0f,
          .k_theta = 1.0f,
          .k_omega = 1.0f,
          .k_i = 0.2f,
          .theta_integral_limit_rad_s = 0.5f,
          .tau_coulomb_nm = 0.05f,
          .coulomb_smooth_rad_s = 0.2f,
          .eso_bandwidth_rad_s = 30.0f,
          .eso_comp_gain = 1.0f,
          .eso_comp_limit_nm = 0.3f,
          .eso_omega_gate_rad_s = 5.0f,
          .eso_alpha_gate_rad_s2 = 50.0f,
          .tau_bias_ki = 0.5f,
          .tau_bias_limit_nm = 0.15f,
          .tau_meas_lpf_alpha = 0.1f,
          .theta_deadband_rad = 0.0f,
          .torque_soft_limit_nm = 2.0f,
          .torque_min_nm = -2.223f,
          .torque_max_nm = 2.223f,
          .torque_slew_rate_nm_s = 1000.0f,
          .eso_enable = true,
          .eso_comp_enable = false,
          .coulomb_enable = false,
          .lqi_enable = false,
          .torque_bias_enable = false,
          .torque_slew_enable = true};
}

#include <limits>

#include "libxr_def.hpp"
#include "yaw_smc_test_support.hpp"

static YawSmc::Output calculate_once(YawSmc& controller,
                                     const YawSmc::Config& config,
                                     float theta_ref, float omega_ref,
                                     float alpha_ref, float theta, float omega,
                                     float dt_s = 0.002f) {
  return controller.Calculate(
      config,
      {.theta_rad = theta_ref,
       .omega_rad_s = omega_ref,
       .alpha_rad_s2 = alpha_ref},
      {.theta_rad = theta, .omega_rad_s = omega, .valid = true}, dt_s);
}

static void test_config_validation() {
  auto cfg = base_yaw_smc_config();
  CHECK(YawSmc::ValidateConfig(cfg));
  cfg.j_kg_m2 = 0.0f;
  CHECK(!YawSmc::ValidateConfig(cfg));
  cfg = base_yaw_smc_config();
  cfg.c = 0.0f;
  CHECK(!YawSmc::ValidateConfig(cfg));
  cfg = base_yaw_smc_config();
  cfg.k = -1.0f;
  CHECK(!YawSmc::ValidateConfig(cfg));
  cfg = base_yaw_smc_config();
  cfg.q = 27.0f;
  CHECK(!YawSmc::ValidateConfig(cfg));
  cfg = base_yaw_smc_config();
  cfg.ftsmc_switch_rad = 0.0f;
  CHECK(!YawSmc::ValidateConfig(cfg));
  cfg = base_yaw_smc_config();
  cfg.sat_boundary = 0.0f;
  CHECK(!YawSmc::ValidateConfig(cfg));
  cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.q = -1.0f;
  cfg.p = 0.0f;
  cfg.ftsmc_switch_rad = 0.0f;
  CHECK(YawSmc::ValidateConfig(cfg));
  cfg = base_yaw_smc_config();
  cfg.epsilon = std::numeric_limits<float>::quiet_NaN();
  CHECK(!YawSmc::ValidateConfig(cfg));
}

static void test_linear_smc_law() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.torque_slew_enable = false;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 3.0f, 0.1f, 0.2f);
  const auto oracle = smc_code_oracle(
      cfg, {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 3.0f},
      {.theta_rad = 0.1f, .omega_rad_s = 0.2f, .valid = true});
  CHECK(output.valid);
  CHECK(!output.used_ftsmc);
  CHECK_NEAR(output.e_theta_rad, oracle.e_theta_rad, 1.0e-6f);
  CHECK_NEAR(output.e_omega_rad_s, oracle.e_omega_rad_s, 1.0e-6f);
  CHECK_NEAR(output.s, oracle.s, 1.0e-6f);
  CHECK_NEAR(output.sat_s, oracle.sat_s, 1.0e-7f);
  CHECK_NEAR(output.tau_ff_alpha_nm, oracle.tau_ff_alpha_nm, 1.0e-6f);
  CHECK_NEAR(output.tau_smc_nm, oracle.tau_smc_nm, 1.0e-5f);
  CHECK_NEAR(output.tau_pre_limit_nm, oracle.tau_pre_limit_nm, 1.0e-5f);
  CHECK_NEAR(output.tau_cmd_nm, output.tau_pre_limit_nm, 1.0e-6f);

  output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f);
  const float S_INNER = 20.0f * 0.01f;
  CHECK_NEAR(output.s, S_INNER, 1.0e-6f);
  CHECK_NEAR(output.sat_s, S_INNER, 1.0e-6f);
  CHECK_NEAR(output.tau_smc_nm, 0.03f * (-0.5f * S_INNER - 2.0f * S_INNER),
             1.0e-5f);

  output = calculate_once(controller, cfg, 0.1f, 0.0f, 99.0f, 0.1f, 0.0f);
  const auto target_history_oracle = smc_code_oracle(
      cfg, {.theta_rad = 0.1f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 99.0f},
      {.theta_rad = 0.1f, .omega_rad_s = 0.0f, .valid = true});
  CHECK_NEAR(output.e_omega_rad_s, target_history_oracle.e_omega_rad_s,
             1.0e-6f);
  CHECK_NEAR(output.tau_ff_alpha_nm, target_history_oracle.tau_ff_alpha_nm,
             1.0e-6f);
  CHECK_NEAR(output.tau_pre_limit_nm, target_history_oracle.tau_pre_limit_nm,
             1.0e-5f);
}

static void test_ftsmc_law_both_signs() {
  auto cfg = base_yaw_smc_config();
  cfg.torque_slew_enable = false;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 3.0f, 0.1f, 0.2f);
  const auto positive_oracle = smc_code_oracle(
      cfg, {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 3.0f},
      {.theta_rad = 0.1f, .omega_rad_s = 0.2f, .valid = true});
  CHECK(output.valid);
  CHECK(output.used_ftsmc);
  CHECK_NEAR(output.s, positive_oracle.s, 1.0e-5f);
  CHECK_NEAR(output.tau_smc_nm, positive_oracle.tau_smc_nm, 1.0e-5f);

  output = calculate_once(controller, cfg, 0.0f, 0.0f, 3.0f, -0.1f, 0.2f);
  const auto negative_oracle = smc_code_oracle(
      cfg, {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 3.0f},
      {.theta_rad = -0.1f, .omega_rad_s = 0.2f, .valid = true});
  CHECK(output.used_ftsmc);
  CHECK(negative_oracle.s < 0.0f);
  CHECK_NEAR(output.s, negative_oracle.s, 1.0e-5f);
  CHECK_NEAR(output.tau_smc_nm, negative_oracle.tau_smc_nm, 1.0e-5f);
}

static void test_ftsmc_switch_and_direct_error() {
  auto cfg = base_yaw_smc_config();
  cfg.torque_slew_enable = false;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.02f, 0.0f);
  CHECK(output.used_ftsmc);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f,
                          cfg.ftsmc_switch_rad, 0.0f);
  CHECK(output.used_ftsmc);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f);
  CHECK(!output.used_ftsmc);

  controller.Reset(3.13f, 0.0f, 0.0f);
  output = calculate_once(controller, cfg, -3.13f, 0.0f, 0.0f, 3.13f, 0.0f);
  CHECK(std::fabs(output.e_theta_rad) < 0.03f);
}

static void test_wrap_and_complete_deadband() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.error_deadband_rad = 0.05f;
  cfg.torque_slew_enable = false;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, 0.1f, 1.0e-6f);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, -0.1f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, -0.1f, 1.0e-6f);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 2.0f, 0.039f, 0.0f);
  CHECK(output.valid);
  CHECK_NEAR(output.e_theta_rad, 0.039f, 1.0e-7f);
  CHECK_NEAR(output.tau_ff_alpha_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_smc_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_pre_limit_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_cmd_nm, 0.0f, 1.0e-7f);
  CHECK(!output.used_ftsmc && !output.soft_limit_active &&
        !output.hard_limit_active && !output.slew_limit_active);
  cfg.error_deadband_rad = 0.04f;
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 2.0f, 0.04f, 0.0f);
  CHECK(output.valid);
  CHECK_NEAR(output.tau_cmd_nm, 0.0f, 1.0e-6f);
  cfg.error_deadband_rad = 0.0f;
  output = calculate_once(controller, cfg, 3.13f, 0.0f, 0.0f, -3.13f, 0.0f);
  CHECK(std::fabs(output.e_theta_rad) < 0.03f);
}

static void test_cycle_value_error_matches_pid_geometry() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.torque_slew_enable = false;

  YawSmc controller;
  const float THETA = -0.5f;
  const float TARGET = static_cast<float>(LibXR::TWO_PI) - 0.5f;
  controller.Reset(THETA, 0.0f, 0.0f);
  auto output =
      calculate_once(controller, cfg, TARGET, 0.0f, 0.0f, THETA, 0.0f);
  CHECK(output.valid);
  CHECK(std::fabs(output.e_theta_rad) < 1.0e-5f);
}

static void test_reference_kinematics_are_consumed_directly() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.epsilon = 0.0f;
  cfg.torque_soft_limit_nm = 0.0f;
  cfg.torque_min_nm = 0.0f;
  cfg.torque_max_nm = 0.0f;
  cfg.torque_slew_enable = false;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  const auto output =
      calculate_once(controller, cfg, 0.1f, 0.4f, 3.0f, 0.2f, 0.7f);

  const float E_THETA_RAD = 0.1f;
  const float E_OMEGA_RAD_S = 0.3f;
  const float S = E_OMEGA_RAD_S + cfg.c * E_THETA_RAD;
  const float EXPECTED_TAU_FF_NM = cfg.j_kg_m2 * 3.0f;
  const float EXPECTED_TAU_SMC_NM =
      cfg.j_kg_m2 * (-cfg.c * E_OMEGA_RAD_S - cfg.k * S);

  CHECK(output.valid);
  CHECK_NEAR(output.e_theta_rad, E_THETA_RAD, 1.0e-6f);
  CHECK_NEAR(output.e_omega_rad_s, E_OMEGA_RAD_S, 1.0e-6f);
  CHECK_NEAR(output.tau_ff_alpha_nm, EXPECTED_TAU_FF_NM, 1.0e-6f);
  CHECK_NEAR(output.tau_smc_nm, EXPECTED_TAU_SMC_NM, 1.0e-6f);
  CHECK_NEAR(output.tau_pre_limit_nm, EXPECTED_TAU_FF_NM + EXPECTED_TAU_SMC_NM,
             1.0e-6f);
}

static void test_core_law_is_independent_of_valid_dt() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.torque_slew_enable = false;

  YawSmc fast_controller;
  YawSmc slow_controller;
  fast_controller.Reset(0.0f, 0.0f, 0.0f);
  slow_controller.Reset(0.0f, 0.0f, 0.0f);

  const auto fast = calculate_once(fast_controller, cfg, 0.1f, 0.4f, 3.0f, 0.2f,
                                   0.7f, 0.001f);
  const auto slow = calculate_once(slow_controller, cfg, 0.1f, 0.4f, 3.0f, 0.2f,
                                   0.7f, 0.010f);

  CHECK(fast.valid && slow.valid);
  CHECK_NEAR(fast.e_omega_rad_s, slow.e_omega_rad_s, 1.0e-7f);
  CHECK_NEAR(fast.tau_ff_alpha_nm, slow.tau_ff_alpha_nm, 1.0e-7f);
  CHECK_NEAR(fast.tau_pre_limit_nm, slow.tau_pre_limit_nm, 1.0e-7f);
}

static void test_ftsmc_feedback_is_odd_symmetric() {
  auto cfg = base_yaw_smc_config();
  cfg.torque_soft_limit_nm = 0.0f;
  cfg.torque_min_nm = 0.0f;
  cfg.torque_max_nm = 0.0f;
  cfg.torque_slew_enable = false;

  YawSmc positive_controller;
  YawSmc negative_controller;
  positive_controller.Reset(0.0f, 0.0f, 0.0f);
  negative_controller.Reset(0.0f, 0.0f, 0.0f);

  const auto positive =
      calculate_once(positive_controller, cfg, 0.0f, 0.0f, 0.0f, 0.1f, 0.2f);
  const auto negative =
      calculate_once(negative_controller, cfg, 0.0f, 0.0f, 0.0f, -0.1f, -0.2f);

  CHECK(positive.valid && negative.valid);
  CHECK(positive.used_ftsmc && negative.used_ftsmc);
  CHECK_NEAR(negative.s, -positive.s, 2.0e-5f);
  CHECK_NEAR(negative.tau_smc_nm, -positive.tau_smc_nm, 1.0e-6f);
}

static void test_ftsmc_negative_error_uses_unsigned_power_derivative() {
  auto cfg = base_yaw_smc_config();
  cfg.torque_soft_limit_nm = 0.0f;
  cfg.torque_min_nm = 0.0f;
  cfg.torque_max_nm = 0.0f;
  cfg.torque_slew_enable = false;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  const auto output =
      calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, -0.1f, 0.2f);

  const float R = cfg.q / cfg.p;
  const float E_THETA_RAD = -0.1f;
  const float E_OMEGA_RAD_S = 0.2f;
  const float S =
      E_OMEGA_RAD_S +
      cfg.c * std::copysign(std::pow(std::fabs(E_THETA_RAD), R), E_THETA_RAD);
  const float SURFACE_DOT_TERM =
      cfg.c * R * std::pow(std::fabs(E_THETA_RAD), R - 1.0f) * E_OMEGA_RAD_S;
  const float EXPECTED_TAU_SMC_NM =
      cfg.j_kg_m2 *
      (-SURFACE_DOT_TERM - cfg.epsilon * sat(S / cfg.sat_boundary) - cfg.k * S);

  CHECK(output.valid && output.used_ftsmc);
  CHECK_NEAR(output.tau_smc_nm, EXPECTED_TAU_SMC_NM, 1.0e-5f);
}

static void test_soft_and_hard_limit_order() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.torque_slew_enable = false;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output =
      calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK(output.tau_pre_limit_nm > 2.0f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 2.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 2.0f, 1.0e-6f);
  CHECK(output.soft_limit_active);
  CHECK(!output.hard_limit_active);

  cfg.torque_soft_limit_nm = 0.0f;
  cfg.torque_max_nm = 1.5f;
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 1.5f, 1.0e-6f);
  CHECK(!output.soft_limit_active);
  CHECK(output.hard_limit_active);
}

static void test_slew_uses_only_committed_torque() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.torque_slew_rate_nm_s = 100.0f;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 1.5f);
  auto output =
      calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK(output.tau_pre_limit_nm > 2.0f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 2.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);
  CHECK(output.slew_limit_active);

  output = calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);

  controller.CommitAppliedTorque(std::numeric_limits<float>::quiet_NaN());
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);

  controller.CommitAppliedTorque(1.7f);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.9f, 1.0e-6f);
}

static void test_slew_reentry_uses_latest_applied_torque() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.torque_slew_enable = false;
  cfg.torque_slew_rate_nm_s = 100.0f;

  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 0.5f);
  auto output =
      calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK_NEAR(output.tau_cmd_nm, 2.0f, 1.0e-6f);
  controller.CommitAppliedTorque(-0.5f);

  cfg.torque_slew_enable = true;
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK_NEAR(output.tau_cmd_nm, -0.3f, 1.0e-6f);
  CHECK(output.slew_limit_active);
}

static void test_invalid_inputs_are_rejected() {
  auto cfg = base_yaw_smc_config();
  cfg.torque_slew_rate_nm_s = 100.0f;
  YawSmc controller;
  controller.Reset(0.0f, 0.0f, 1.5f);

  auto invalid_cfg = cfg;
  invalid_cfg.c = -1.0f;
  auto output = calculate_once(controller, invalid_cfg, 0.0f, -100.0f, 200.0f,
                               0.1f, 0.0f);
  CHECK(!output.valid);

  output = controller.Calculate(
      cfg,
      {.theta_rad = std::numeric_limits<float>::quiet_NaN(),
       .omega_rad_s = 0.0f,
       .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .valid = true}, 0.002f);
  CHECK(!output.valid);

  output = controller.Calculate(
      cfg, {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .valid = false}, 0.002f);
  CHECK(!output.valid);

  output =
      calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0005f);
  CHECK(!output.valid);
  output =
      calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.020f);
  CHECK(output.valid);
  output =
      calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.020001f);
  CHECK(!output.valid);

  output = calculate_once(controller, cfg, 0.0f, 0.0f, 200.0f, 0.1f, -100.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);
}

int main() {
  test_config_validation();
  test_linear_smc_law();
  test_ftsmc_law_both_signs();
  test_ftsmc_switch_and_direct_error();
  test_wrap_and_complete_deadband();
  test_cycle_value_error_matches_pid_geometry();
  test_reference_kinematics_are_consumed_directly();
  test_core_law_is_independent_of_valid_dt();
  test_ftsmc_feedback_is_odd_symmetric();
  test_ftsmc_negative_error_uses_unsigned_power_derivative();
  test_soft_and_hard_limit_order();
  test_slew_uses_only_committed_torque();
  test_slew_reentry_uses_latest_applied_torque();
  test_invalid_inputs_are_rejected();
  if (yaw_smc_test_failures != 0) {
    std::fprintf(stderr, "%d YawSmc checks failed\n", yaw_smc_test_failures);
    return 1;
  }
  return 0;
}

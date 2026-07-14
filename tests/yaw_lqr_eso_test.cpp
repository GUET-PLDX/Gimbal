#include <limits>

#include "yaw_lqr_eso_test_support.hpp"

static YawLqrEso::Output calculate_once(YawLqrEso& controller,
                                        const YawLqrEso::Config& config,
                                        float theta_ref, float omega_ref,
                                        float alpha_ref, float theta,
                                        float omega) {
  return controller.Calculate(config,
                              {.theta_rad = theta_ref,
                               .omega_rad_s = omega_ref,
                               .alpha_rad_s2 = alpha_ref},
                              {.theta_rad = theta,
                               .omega_rad_s = omega,
                               .tau_meas_nm = 0.0f,
                               .valid = true,
                               .torque_measurement_valid = true},
                              0.002f);
}

static void test_config_validation() {
  auto cfg = base_yaw_config();
  CHECK(YawLqrEso::ValidateConfig(cfg));
  cfg.j_kg_m2 = 0.0f;
  CHECK(!YawLqrEso::ValidateConfig(cfg));
  cfg = base_yaw_config();
  cfg.k_theta = -1.0f;
  CHECK(!YawLqrEso::ValidateConfig(cfg));
  cfg = base_yaw_config();
  cfg.torque_bias_enable = true;
  cfg.tau_meas_lpf_alpha = 1.1f;
  CHECK(!YawLqrEso::ValidateConfig(cfg));
  cfg = base_yaw_config();
  cfg.k_omega = std::numeric_limits<float>::quiet_NaN();
  CHECK(!YawLqrEso::ValidateConfig(cfg));
}

static void test_base_state_feedback_and_angle_unwrap() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_enable = false;

  YawLqrEso controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output = calculate_once(controller, cfg, 0.1f, 0.2f, 3.0f, 0.0f, 0.0f);
  CHECK(output.valid);
  CHECK_NEAR(output.e_theta_rad, -0.1f, 1.0e-6f);
  CHECK_NEAR(output.e_omega_rad_s, -0.2f, 1.0e-6f);
  CHECK_NEAR(output.tau_ff_alpha_nm, 0.09f, 1.0e-6f);
  CHECK_NEAR(output.tau_lqr_nm, 0.39f, 1.0e-5f);
  CHECK_NEAR(output.tau_pre_limit_nm, 0.39f, 1.0e-5f);
  CHECK_NEAR(output.tau_cmd_nm, 0.39f, 1.0e-5f);
  CHECK_NEAR(output.tau_ff_coulomb_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_lqi_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_eso_active_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_bias_nm, 0.0f, 1.0e-7f);

  controller.Reset(3.13f, 0.0f, 0.0f);
  output = calculate_once(controller, cfg, -3.13f, 0.0f, 0.0f, -3.13f, 0.0f);
  CHECK(std::fabs(output.e_theta_rad) < 0.03f);
  CHECK(output.theta_unwrapped_rad > 3.14f);
}

static void test_signed_angle_deadband() {
  auto cfg = base_yaw_config();
  cfg.theta_deadband_rad = 0.05f;
  cfg.torque_slew_enable = false;

  YawLqrEso controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, 0.05f, 1.0e-6f);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, -0.1f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, -0.05f, 1.0e-6f);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 0.0f, 0.04f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, 0.0f, 1.0e-7f);
}

static void test_soft_and_hard_limit_order() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_enable = false;

  YawLqrEso controller;
  controller.Reset(0.0f, 0.0f, 0.0f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_pre_limit_nm, 3.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 2.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 2.0f, 1.0e-6f);
  CHECK(output.soft_limit_active);
  CHECK(!output.hard_limit_active);
  CHECK(!output.slew_limit_active);

  output = calculate_once(controller, cfg, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_pre_limit_nm, 1.5f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 1.5f, 1.0e-6f);
  CHECK(!output.soft_limit_active);

  cfg.torque_soft_limit_nm = 0.0f;
  cfg.torque_max_nm = 1.5f;
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_pre_limit_nm, 3.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 1.5f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 1.5f, 1.0e-6f);
  CHECK(!output.soft_limit_active);
  CHECK(output.hard_limit_active);
}

static void test_slew_uses_only_committed_torque() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_rate_nm_s = 100.0f;

  YawLqrEso controller;
  controller.Reset(0.0f, 0.0f, 1.5f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_pre_limit_nm, 3.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 2.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);
  CHECK(output.soft_limit_active);
  CHECK(output.slew_limit_active);

  output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);

  controller.CommitAppliedTorque(std::numeric_limits<float>::quiet_NaN());
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);

  controller.CommitAppliedTorque(1.7f);
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.9f, 1.0e-6f);

  cfg.torque_max_nm = 1.0f;
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK(output.tau_cmd_nm <= 1.0f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 1.0f, 1.0e-6f);
  CHECK(output.hard_limit_active);
}

static void test_slew_reentry_uses_latest_applied_torque() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_enable = false;
  cfg.torque_slew_rate_nm_s = 100.0f;

  YawLqrEso controller;
  controller.Reset(0.0f, 0.0f, 0.5f);
  auto output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 2.0f, 1.0e-6f);
  controller.CommitAppliedTorque(-0.5f);

  cfg.torque_slew_enable = true;
  output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, -0.3f, 1.0e-6f);
  CHECK(output.slew_limit_active);
}

static void test_invalid_inputs_are_rejected() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_rate_nm_s = 100.0f;
  YawLqrEso controller;
  controller.Reset(0.0f, 0.0f, 1.5f);

  auto invalid_cfg = cfg;
  invalid_cfg.k_theta = -1.0f;
  auto output =
      calculate_once(controller, invalid_cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK(!output.valid);

  output = controller.Calculate(
      cfg,
      {.theta_rad = std::numeric_limits<float>::quiet_NaN(),
       .omega_rad_s = 0.0f,
       .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = true,
       .torque_measurement_valid = true},
      0.002f);
  CHECK(!output.valid);

  output = controller.Calculate(
      cfg, {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = false,
       .torque_measurement_valid = true},
      0.002f);
  CHECK(!output.valid);

  output = controller.Calculate(
      cfg, {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = true,
       .torque_measurement_valid = true},
      0.0005f);
  CHECK(!output.valid);

  output = calculate_once(controller, cfg, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);
}

int main() {
  test_config_validation();
  test_base_state_feedback_and_angle_unwrap();
  test_signed_angle_deadband();
  test_soft_and_hard_limit_order();
  test_slew_uses_only_committed_torque();
  test_slew_reentry_uses_latest_applied_torque();
  test_invalid_inputs_are_rejected();
  return yaw_test_failures == 0 ? 0 : 1;
}

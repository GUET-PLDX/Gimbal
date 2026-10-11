#include <limits>
#include <type_traits>
#include <utility>

#include "yaw_lqr_eso_test_support.hpp"

// 接口契约：控制器必须携带显式配置；配置锁存后实例不再可赋值。
static_assert(!std::is_default_constructible_v<YawLqrEso>);
static_assert(std::is_copy_constructible_v<YawLqrEso>);
static_assert(!std::is_copy_assignable_v<YawLqrEso>);
static_assert(!std::is_move_assignable_v<YawLqrEso>);
static_assert(
    std::is_same_v<decltype(std::declval<const YawLqrEso&>().GetConfig()),
                   const YawLqrEso::Config&>);

static YawLqrEso::Output calculate_once(
    YawLqrEso& controller, float theta_ref, float omega_ref, float alpha_ref,
    float theta, float omega,
    float torque_limit_nm = TEST_YAW_TORQUE_LIMIT_NM) {
  return controller.Calculate({.theta_rad = theta_ref,
                               .omega_rad_s = omega_ref,
                               .alpha_rad_s2 = alpha_ref},
                              {.theta_rad = theta,
                               .omega_rad_s = omega,
                               .tau_meas_nm = 0.0f,
                               .valid = true,
                               .torque_measurement_valid = true},
                              0.002f, TEST_YAW_J_KG_M2, torque_limit_nm,
                              TEST_YAW_B_NMS_RAD);
}

static YawLqrEso::Output calculate_with_torque(YawLqrEso& controller,
                                               float theta_ref, float omega_ref,
                                               float alpha_ref, float theta,
                                               float omega, float tau_meas_nm,
                                               bool torque_measurement_valid) {
  return controller.Calculate(
      {.theta_rad = theta_ref,
       .omega_rad_s = omega_ref,
       .alpha_rad_s2 = alpha_ref},
      {.theta_rad = theta,
       .omega_rad_s = omega,
       .tau_meas_nm = tau_meas_nm,
       .valid = true,
       .torque_measurement_valid = torque_measurement_valid},
      0.002f, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD);
}

static void test_config_validation() {
  auto cfg = base_yaw_config();
  CHECK(YawLqrEso::ValidateConfig(
      cfg, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD));
  CHECK(YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2, 0.0f,
                                  TEST_YAW_B_NMS_RAD));
  CHECK(!YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2, -1.0f,
                                   TEST_YAW_B_NMS_RAD));
  CHECK(!YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2,
                                   std::numeric_limits<float>::quiet_NaN(),
                                   TEST_YAW_B_NMS_RAD));
  CHECK(!YawLqrEso::ValidateConfig(cfg, std::numeric_limits<float>::quiet_NaN(),
                                   TEST_YAW_TORQUE_LIMIT_NM,
                                   TEST_YAW_B_NMS_RAD));
  CHECK(!YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2,
                                   TEST_YAW_TORQUE_LIMIT_NM,
                                   std::numeric_limits<float>::quiet_NaN()));
  CHECK(!YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2,
                                   TEST_YAW_TORQUE_LIMIT_NM, -1.0f));
  cfg.k_theta = -1.0f;
  CHECK(!YawLqrEso::ValidateConfig(
      cfg, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD));
  cfg = base_yaw_config();
  cfg.torque_bias_enable = true;
  cfg.tau_meas_lpf_alpha = 1.1f;
  CHECK(!YawLqrEso::ValidateConfig(
      cfg, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD));
  cfg = base_yaw_config();
  cfg.k_omega = std::numeric_limits<float>::quiet_NaN();
  CHECK(!YawLqrEso::ValidateConfig(
      cfg, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD));
  // 反馈增益必须严格为正：两者同时为 0 时角度环与角速度环一起失效，控制器会静默
  // 输出零力矩（"配置被整块清零 / 聚合实参被截断"就是这个形状）。
  cfg = base_yaw_config();
  cfg.k_theta = 0.0f;
  CHECK(!YawLqrEso::ValidateConfig(
      cfg, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD));
  cfg = base_yaw_config();
  cfg.k_omega = 0.0f;
  CHECK(!YawLqrEso::ValidateConfig(
      cfg, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD));
  cfg = base_yaw_config();
  cfg.k_theta = 0.0f;
  cfg.k_omega = 0.0f;
  CHECK(!YawLqrEso::ValidateConfig(
      cfg, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD));
  // 默认构造的 Config 必须是**可用**配置（逐字段等于 Gimbal.hpp 的 manifest
  // 默认值， 由 tests/gimbal_config_order_regression.py
  // 守护），不是"未配置"哨兵。
  CHECK(YawLqrEso::ValidateConfig(YawLqrEso::Config{}, TEST_YAW_J_KG_M2,
                                  TEST_YAW_TORQUE_LIMIT_NM,
                                  TEST_YAW_B_NMS_RAD));
}

static void test_base_state_feedback_and_angle_unwrap() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  auto output = calculate_once(controller, 0.1f, 0.2f, 3.0f, 0.0f, 0.0f);
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
  output = calculate_once(controller, -3.13f, 0.0f, 0.0f, -3.13f, 0.0f);
  CHECK(std::fabs(output.e_theta_rad) < 0.03f);
  CHECK(output.theta_unwrapped_rad > static_cast<float>(LibXR::PI));
}

static void test_signed_angle_deadband() {
  auto cfg = base_yaw_config();
  cfg.theta_deadband_rad = 0.05f;
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  auto output = calculate_once(controller, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, 0.05f, 1.0e-6f);
  output = calculate_once(controller, 0.0f, 0.0f, 0.0f, -0.1f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, -0.05f, 1.0e-6f);
  output = calculate_once(controller, 0.0f, 0.0f, 0.0f, 0.04f, 0.0f);
  CHECK_NEAR(output.e_theta_rad, 0.0f, 1.0e-7f);
}

static void test_soft_and_hard_limit_order() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  auto output = calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_pre_limit_nm, 3.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 2.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 2.0f, 1.0e-6f);
  CHECK(output.soft_limit_active);
  CHECK(!output.hard_limit_active);
  CHECK(!output.slew_limit_active);

  output = calculate_once(controller, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_pre_limit_nm, 1.5f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 1.5f, 1.0e-6f);
  CHECK(!output.soft_limit_active);

  // 关闭软限幅是配置变更，锁存后由新实例表达；硬限幅仍按调用传入。
  cfg.torque_soft_limit_nm = 0.0f;
  YawLqrEso hard_limited_controller{cfg};
  output = calculate_once(hard_limited_controller, 0.0f, 0.0f, 100.0f, 0.0f,
                          0.0f, 1.5f);
  CHECK_NEAR(output.tau_pre_limit_nm, 3.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 1.5f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 1.5f, 1.0e-6f);
  CHECK(!output.soft_limit_active);
  CHECK(output.hard_limit_active);

  output = calculate_once(hard_limited_controller, 0.0f, 0.0f, 100.0f, 0.0f,
                          0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 3.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 3.0f, 1.0e-6f);
  CHECK(!output.hard_limit_active);
}

static void test_slew_uses_only_committed_torque() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_rate_nm_s = 100.0f;

  YawLqrEso controller{cfg};
  controller.Reset(0.0f, 0.0f, 1.5f);
  auto output = calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_pre_limit_nm, 3.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 2.0f, 1.0e-6f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);
  CHECK(output.soft_limit_active);
  CHECK(output.slew_limit_active);

  output = calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);

  controller.CommitAppliedTorque(std::numeric_limits<float>::quiet_NaN());
  output = calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);

  controller.CommitAppliedTorque(1.7f);
  output = calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.9f, 1.0e-6f);

  output = calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 1.0f);
  CHECK(output.tau_cmd_nm <= 1.0f);
  CHECK_NEAR(output.tau_cmd_before_slew_nm, 1.0f, 1.0e-6f);
  CHECK(output.hard_limit_active);
}

// 原场景是"运行期重新使能 slew"；配置锁存后该路径不可表达，改为等价的
// "首周期 slew anchor 取自最近一次已施加力矩"（Reset 播种 -0.5 N*m）。
static void test_slew_reentry_uses_latest_applied_torque() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_rate_nm_s = 100.0f;

  YawLqrEso controller{cfg};
  controller.Reset(0.0f, 0.0f, -0.5f);
  const auto output =
      calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, -0.3f, 1.0e-6f);
  CHECK(output.slew_limit_active);
}

static void test_coulomb_feedforward_switch() {
  auto cfg = base_yaw_config();
  cfg.coulomb_enable = true;
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  auto output = calculate_once(controller, 0.0f, 0.2f, 0.0f, 0.0f, 0.2f);
  CHECK_NEAR(output.tau_ff_coulomb_nm, 0.05f * std::tanh(1.0f), 1.0e-6f);
  CHECK_NEAR(output.tau_lqr_nm, output.tau_ff_coulomb_nm, 1.0e-7f);

  cfg.coulomb_enable = false;
  YawLqrEso no_coulomb_controller{cfg};
  output = calculate_once(no_coulomb_controller, 0.0f, 0.2f, 0.0f, 0.0f, 0.2f);
  CHECK_NEAR(output.tau_ff_coulomb_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_lqr_nm, 0.0f, 1.0e-7f);
}

static void test_lqi_integral_limit_and_falling_edge() {
  auto cfg = base_yaw_config();
  cfg.lqi_enable = true;
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  controller.Reset(0.1f, 0.0f, 0.0f);
  YawLqrEso::Output output{};
  for (int i = 0; i < 1000; ++i) {
    output = calculate_once(controller, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
  }
  CHECK(std::fabs(output.tau_lqi_nm) <= 0.100001f);
  CHECK_NEAR(output.tau_lqi_nm, -0.04f, 1.0e-5f);

  // 关闭 LQI 后积分清零由"不带 LQI 的实例"表达。
  cfg.lqi_enable = false;
  YawLqrEso no_lqi_controller{cfg};
  output = calculate_once(no_lqi_controller, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
  CHECK_NEAR(output.tau_lqi_nm, 0.0f, 1.0e-7f);

  // 重新使能后积分从 0 起算 —— 等价于新实例（构造期 Reset 已把积分清零）。
  cfg.lqi_enable = true;
  YawLqrEso restarted_lqi_controller{cfg};
  output =
      calculate_once(restarted_lqi_controller, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
  CHECK_NEAR(output.tau_lqi_nm, -0.00004f, 1.0e-7f);
}

static void test_bias_measurement_validation_and_lpf_initialization() {
  auto cfg = base_yaw_config();
  cfg.eso_enable = false;
  cfg.torque_bias_enable = true;
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  auto output = calculate_with_torque(controller, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                                      0.3f, true);
  CHECK(output.valid);
  CHECK_NEAR(output.tau_bias_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.tau_pre_limit_nm, 0.3f, 1.0e-6f);

  output = calculate_with_torque(controller, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                                 0.0f, true);
  CHECK_NEAR(output.tau_bias_nm, 0.00003f, 1.0e-7f);

  cfg.torque_bias_enable = false;
  YawLqrEso no_bias_controller{cfg};
  output =
      calculate_with_torque(no_bias_controller, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                            std::numeric_limits<float>::quiet_NaN(), false);
  CHECK(output.valid);
  CHECK_NEAR(output.tau_bias_nm, 0.0f, 1.0e-7f);

  // 非法力矩测量在校验层就被拒绝，与是否启用 bias 无关。
  cfg.torque_bias_enable = true;
  YawLqrEso bias_controller{cfg};
  output = calculate_with_torque(bias_controller, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                                 std::numeric_limits<float>::quiet_NaN(), true);
  CHECK(!output.valid);
  output = calculate_with_torque(bias_controller, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                                 0.3f, false);
  CHECK(!output.valid);
}

static void test_bias_limit_and_falling_edge_reset() {
  auto cfg = base_yaw_config();
  cfg.eso_enable = false;
  cfg.torque_bias_enable = true;
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  YawLqrEso::Output output{};
  for (int i = 0; i < 1000; ++i) {
    output = calculate_with_torque(controller, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                   0.0f, true);
  }
  CHECK_NEAR(output.tau_bias_nm, 0.15f, 1.0e-6f);

  // 关闭 bias 会把估计器清零，由"不带 bias 的实例"表达。
  cfg.torque_bias_enable = false;
  YawLqrEso no_bias_controller{cfg};
  output =
      calculate_with_torque(no_bias_controller, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            std::numeric_limits<float>::quiet_NaN(), false);
  CHECK(output.valid);
  CHECK_NEAR(output.tau_bias_nm, 0.0f, 1.0e-7f);

  // 重新使能后偏置从 0 起算（构造期 Reset 已清零），且首周期重新播种 LPF。
  cfg.torque_bias_enable = true;
  YawLqrEso restarted_bias_controller{cfg};
  output = calculate_with_torque(restarted_bias_controller, 1.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 1.0f, true);
  CHECK_NEAR(output.tau_bias_nm, 0.0f, 1.0e-7f);
}

// 原 test_slew_falling_edge_restarts_from_latest_commit 覆盖的是
// "关闭 slew → 再打开 slew，anchor 重新取最近一次已提交力矩"。
// 配置锁存后 slew 不可能在运行期被重新使能，该"下降沿后重启"路径已不存在；
// 其残留不变量（首周期 anchor 取自已提交力矩）由
// test_slew_reentry_uses_latest_applied_torque 与
// test_slew_uses_only_committed_torque 覆盖，故此处不再保留同名场景。

static void test_eso_fresh_cycle_and_simultaneous_euler() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  controller.Reset(0.2f, 0.4f, 0.6f);
  auto output = calculate_once(controller, 0.2f, 0.4f, 30.0f, 0.2f, 0.4f);
  CHECK(output.valid);
  CHECK_NEAR(output.tau_cmd_nm, 0.9f, 1.0e-6f);
  CHECK_NEAR(output.z1, 0.2f, 1.0e-7f);
  CHECK_NEAR(output.z2, 0.4f, 1.0e-7f);
  CHECK_NEAR(output.z3, 0.0f, 1.0e-7f);
  CHECK(!output.observer_ready);

  output = calculate_once(controller, 0.3f, 0.4f, 0.0f, 0.3f, 0.4f);
  const float B0 = 1.0f / TEST_YAW_J_KG_M2;
  const float BETA1 = 3.0f * cfg.eso_bandwidth_rad_s;
  const float BETA2 = 3.0f * cfg.eso_bandwidth_rad_s * cfg.eso_bandwidth_rad_s;
  const float BETA3 = cfg.eso_bandwidth_rad_s * cfg.eso_bandwidth_rad_s *
                      cfg.eso_bandwidth_rad_s;
  const float OBSERVER_ERROR_RAD = 0.1f;
  const float DT_S = 0.002f;
  const float EXPECTED_Z1 = 0.2f + DT_S * (0.4f + BETA1 * OBSERVER_ERROR_RAD);
  const float EXPECTED_Z2 =
      0.4f + DT_S * (B0 * 0.6f + BETA2 * OBSERVER_ERROR_RAD);
  const float EXPECTED_Z3 = DT_S * BETA3 * OBSERVER_ERROR_RAD;
  CHECK(output.observer_ready);
  CHECK_NEAR(output.z1, EXPECTED_Z1, 1.0e-6f);
  CHECK_NEAR(output.z2, EXPECTED_Z2, 1.0e-5f);
  CHECK_NEAR(output.z3, EXPECTED_Z3, 1.0e-5f);
}

static void test_observer_only_isolation_and_falling_edge() {
  auto observer_cfg = base_yaw_config();
  observer_cfg.torque_slew_enable = false;
  auto observer_off_cfg = observer_cfg;
  observer_off_cfg.eso_enable = false;

  YawLqrEso observer_controller{observer_cfg};
  YawLqrEso observer_off_controller{observer_off_cfg};
  observer_controller.Reset(0.0f, 0.0f, 0.25f);
  observer_off_controller.Reset(0.0f, 0.0f, 0.25f);

  auto observer_output =
      calculate_once(observer_controller, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  auto observer_off_output =
      calculate_once(observer_off_controller, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  CHECK_NEAR(observer_output.tau_cmd_nm, observer_off_output.tau_cmd_nm,
             1.0e-7f);

  observer_output =
      calculate_once(observer_controller, 0.1f, 0.0f, 0.0f, 0.1f, 0.0f);
  observer_off_output =
      calculate_once(observer_off_controller, 0.1f, 0.0f, 0.0f, 0.1f, 0.0f);
  CHECK_NEAR(observer_output.tau_cmd_nm, observer_off_output.tau_cmd_nm,
             1.0e-7f);
  CHECK(std::fabs(observer_output.z3) > 1.0e-6f);
  CHECK_NEAR(observer_off_output.z3, 0.0f, 1.0e-7f);

  // ESO 关闭时 z 直接跟随测量值；配置锁存后不存在"运行期关掉 ESO"，
  // 该语义由专门的 observer_off 实例持续表达。
  observer_off_output =
      calculate_once(observer_off_controller, 0.25f, -0.4f, 0.0f, 0.25f, -0.4f);
  CHECK_NEAR(observer_off_output.z1, 0.25f, 1.0e-6f);
  CHECK_NEAR(observer_off_output.z2, -0.4f, 1.0e-7f);
  CHECK_NEAR(observer_off_output.z3, 0.0f, 1.0e-7f);
  CHECK(!observer_off_output.observer_ready);

  // "ESO 重新使能后必须重新播种"改为"Reset 之后必须重新播种" —— 生产侧
  // ESO 的使能状态只由构造决定，运行期唯一会重置观测器的路径就是 Reset。
  observer_controller.Reset(0.3f, 0.2f, 0.0f);
  observer_output =
      calculate_once(observer_controller, 0.3f, 0.2f, 0.0f, 0.3f, 0.2f);
  CHECK_NEAR(observer_output.z1, 0.3f, 1.0e-6f);
  CHECK_NEAR(observer_output.z2, 0.2f, 1.0e-7f);
  CHECK_NEAR(observer_output.z3, 0.0f, 1.0e-7f);
  CHECK(!observer_output.observer_ready);
}

static void test_eso_compensation_limit_gates_and_switch() {
  auto cfg = base_yaw_config();
  cfg.eso_comp_enable = true;
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  auto output = calculate_once(controller, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  CHECK(!output.eso_comp_active);

  output = calculate_once(controller, 1.0f, -5.0f, -50.0f, 1.0f, -5.0f);
  CHECK(output.eso_comp_active);
  CHECK_NEAR(output.tau_eso_raw_nm, -0.3f, 1.0e-6f);
  CHECK_NEAR(output.tau_eso_active_nm, -0.3f, 1.0e-6f);
  CHECK_NEAR(output.tau_pre_limit_nm, output.tau_lqr_nm - 0.3f, 1.0e-6f);

  output = calculate_once(controller, 1.0f, -5.0001f, 0.0f, 1.0f, -5.0001f);
  CHECK(!output.eso_comp_active);
  CHECK_NEAR(output.tau_eso_active_nm, 0.0f, 1.0e-7f);

  output = calculate_once(controller, 1.0f, 0.0f, 50.0001f, 1.0f, 0.0f);
  CHECK(!output.eso_comp_active);
  CHECK_NEAR(output.tau_eso_active_nm, 0.0f, 1.0e-7f);

  cfg.eso_comp_enable = false;
  YawLqrEso no_comp_controller{cfg};
  output = calculate_once(no_comp_controller, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f);
  CHECK(!output.eso_comp_active);
  CHECK_NEAR(output.tau_eso_active_nm, 0.0f, 1.0e-7f);
}

static void test_non_finite_observer_is_isolated() {
  auto cfg = base_yaw_config();
  cfg.eso_bandwidth_rad_s = std::numeric_limits<float>::max();
  cfg.eso_comp_enable = true;
  cfg.torque_slew_enable = false;

  YawLqrEso controller{cfg};
  auto output = calculate_once(controller, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  CHECK(output.valid);

  output = calculate_once(controller, 0.1f, 0.0f, 0.0f, 0.1f, 0.0f);
  CHECK(output.valid);
  CHECK_NEAR(output.tau_cmd_nm, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.z1, 0.1f, 1.0e-6f);
  CHECK_NEAR(output.z2, 0.0f, 1.0e-7f);
  CHECK_NEAR(output.z3, 0.0f, 1.0e-7f);
  CHECK(!output.observer_ready);
  CHECK(!output.eso_comp_active);
}

static void test_invalid_inputs_are_rejected() {
  auto cfg = base_yaw_config();
  cfg.torque_slew_rate_nm_s = 100.0f;
  YawLqrEso controller{cfg};
  controller.Reset(0.0f, 0.0f, 1.5f);

  // 非法配置在锁存形态下由"另一个用非法配置构造的实例"表达。
  auto invalid_cfg = cfg;
  invalid_cfg.k_theta = -1.0f;
  YawLqrEso invalid_controller{invalid_cfg};
  auto output =
      calculate_once(invalid_controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK(!output.valid);

  output = controller.Calculate(
      {.theta_rad = std::numeric_limits<float>::quiet_NaN(),
       .omega_rad_s = 0.0f,
       .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = true,
       .torque_measurement_valid = true},
      0.002f, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD);
  CHECK(!output.valid);

  output = controller.Calculate(
      {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = false,
       .torque_measurement_valid = true},
      0.002f, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD);
  CHECK(!output.valid);

  output = controller.Calculate(
      {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = true,
       .torque_measurement_valid = true},
      0.0005f, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD);
  CHECK(!output.valid);

  output = controller.Calculate(
      {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = true,
       .torque_measurement_valid = true},
      0.020f, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM, TEST_YAW_B_NMS_RAD);
  CHECK(output.valid);

  output = controller.Calculate(
      {.theta_rad = 0.0f, .omega_rad_s = 0.0f, .alpha_rad_s2 = 0.0f},
      {.theta_rad = 0.0f,
       .omega_rad_s = 0.0f,
       .tau_meas_nm = 0.0f,
       .valid = true,
       .torque_measurement_valid = true},
      0.020001f, TEST_YAW_J_KG_M2, TEST_YAW_TORQUE_LIMIT_NM,
      TEST_YAW_B_NMS_RAD);
  CHECK(!output.valid);

  output = calculate_once(controller, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  CHECK_NEAR(output.tau_cmd_nm, 1.7f, 1.0e-6f);
}

int main() {
  test_config_validation();
  test_base_state_feedback_and_angle_unwrap();
  test_signed_angle_deadband();
  test_soft_and_hard_limit_order();
  test_slew_uses_only_committed_torque();
  test_slew_reentry_uses_latest_applied_torque();
  test_coulomb_feedforward_switch();
  test_lqi_integral_limit_and_falling_edge();
  test_bias_measurement_validation_and_lpf_initialization();
  test_bias_limit_and_falling_edge_reset();
  test_eso_fresh_cycle_and_simultaneous_euler();
  test_observer_only_isolation_and_falling_edge();
  test_eso_compensation_limit_gates_and_switch();
  test_non_finite_observer_is_isolated();
  test_invalid_inputs_are_rejected();
  return yaw_test_failures == 0 ? 0 : 1;
}

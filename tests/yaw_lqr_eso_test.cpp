#include <limits>

#include "yaw_lqr_eso_test_support.hpp"

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

int main() {
  test_config_validation();
  return yaw_test_failures == 0 ? 0 : 1;
}

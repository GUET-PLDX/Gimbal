#include <cmath>
#include <cstdio>

#include "YawSmc.hpp"

int main() {
  const YawSmc::Param config{.c = 2.0f,
                             .k = 3.0f,
                             .epsilon = 1.0f,
                             .q = 1.0f,
                             .p = 2.0f,
                             .error_deadband_rad = 0.125f,
                             .ftsmc_switch_rad = 0.25f};
  const YawSmc controller(config, 2.0f);
  const YawSmc::Reference reference{.omega_rad_s = 0.25f, .alpha_rad_s2 = 0.5f};
  struct Case {
    float angle_rad;
    float expected_torque_nm;
  };
  const Case cases[] = {{0.0625f, 0.0f}, {0.125f, -1.0f}, {0.25f, -2.0f},
                        {-0.390625f, 2.0f}, {0.125f, -1.0f}};
  for (const auto& sample : cases) {
    const float torque_nm = controller.Calculate(
        reference, {.theta_rad = sample.angle_rad, .omega_rad_s = 0.5f}, 0.5f);
    if (!(std::fabs(torque_nm - sample.expected_torque_nm) < 1e-5f)) {
      std::fprintf(stderr, "Yaw error %g: torque %g, expected %g\n",
                   static_cast<double>(sample.angle_rad),
                   static_cast<double>(torque_nm),
                   static_cast<double>(sample.expected_torque_nm));
      return 1;
    }
  }
  return 0;
}

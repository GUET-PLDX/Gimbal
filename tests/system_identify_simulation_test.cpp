// SystemIdentify PC 端仿真回归测试。
//
// 虚拟被控对象：J*alpha + B*omega + tau_c*tanh(omega/w0) [+ mgl*sin(theta)] =
// tau 与 SystemIdentify 的回归模型同构，验证：
//   1. 无噪声：yaw/pitch 辨识值与真值相对误差 < 2%
//   2. 含噪声（陀螺测量噪声 0.02 rad/s 峰值均匀分布）：< 10%
//   3. 变异验证：编译期定义 SYSID_MUTATE_J 把对象惯量改为 1.5 倍而期望不变，
//      测试必须失败（证明断言对 J 敏感，能捕获原始失效模式）。
//
// 构建运行见 system_identify_host_regression.sh。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "SystemIdentify.hpp"

namespace {

constexpr double DT_S = 0.001;
constexpr double PLANT_W0 = 0.1;  // 与 Config.coulomb_tanh_scale 一致

// 确定性伪随机（LCG），避免 <random> 在嵌入式/主机间行为差异。
struct Lcg {
  uint32_t state = 0x12345678U;
  double Uniform() {  // [-1, 1)
    state = state * 1664525U + 1013904223U;
    return static_cast<double>(state) / 2147483648.0 - 1.0;
  }
};

struct Plant {
  double j;
  double b;
  double tau_c;
  double mgl;
  double omega = 0.0;
  double theta = 0.0;

  void Step(double tau, bool pitch) {
    const double GRAVITY = pitch ? mgl * std::sin(theta) : 0.0;
    const double ACC =
        (tau - b * omega - tau_c * std::tanh(omega / PLANT_W0) - GRAVITY) / j;
    omega += ACC * DT_S;
    theta += omega * DT_S;
  }
};

struct Truth {
  double j;
  double b;
  double tau_c;
  double mgl;
};

using AxisResultOut = SystemIdentify::AxisResult;

bool NearlyEqual(double actual, double expected, double rel_tol,
                 const char* name) {
  const double ERR = std::fabs(actual - expected) / std::fabs(expected);
  if (ERR > rel_tol) {
    std::printf("FAIL: %s actual=%.6g expected=%.6g rel_err=%.4f > %.4f\n",
                name, actual, expected, ERR, rel_tol);
    return false;
  }
  return true;
}

bool RunIdentification(const SystemIdentify::Config& config, bool pitch_axis,
                       const Truth& truth, double omega_noise,
                       AxisResultOut& result) {
  SystemIdentify sysid(config);
  sysid.Begin();
  Plant plant{truth.j, truth.b, truth.tau_c, truth.mgl};
  Lcg lcg;

  SystemIdentify::Sensors sensors{};
  sensors.rc_online = true;
  sensors.yaw_motor_online = true;
  sensors.pit_motor_online = true;

  constexpr double MAX_TIME_S = 120.0;
  const uint32_t MAX_STEPS = static_cast<uint32_t>(MAX_TIME_S / DT_S);
  for (uint32_t step = 0; step < MAX_STEPS; step++) {
    const double MEASURED_OMEGA = plant.omega + omega_noise * lcg.Uniform();
    sensors.yaw_omega_rad_s = static_cast<float>(MEASURED_OMEGA);
    sensors.pit_omega_rad_s = static_cast<float>(MEASURED_OMEGA);
    sensors.pit_theta_rad = static_cast<float>(plant.theta);
    const float TAU = sysid.Update(sensors, static_cast<float>(DT_S));
    plant.Step(TAU, pitch_axis);
    if (sysid.State() == IdentifyState::DONE) {
      result = pitch_axis ? sysid.PitResult() : sysid.YawResult();
      return result.converged;
    }
    if (sysid.State() == IdentifyState::ABORTED) {
      std::printf("FAIL: identification aborted, reason=%d\n",
                  static_cast<int>(sysid.AbortReason()));
      return false;
    }
  }
  std::printf("FAIL: identification did not finish within %.0f s\n",
              MAX_TIME_S);
  return false;
}

SystemIdentify::Config MakeConfig(IdentifyAxis axis) {
  SystemIdentify::Config config{};
  config.enabled = true;
  config.axis_select = axis;
  config.arming_delay_s = 0.1f;
  config.step_cycles = 10;
  return config;
}

bool CheckTruth(const AxisResultOut& result, const Truth& truth, double rel_tol,
                bool pitch_axis) {
  bool ok = true;
  ok &= NearlyEqual(result.j, truth.j, rel_tol, "J");
  ok &= NearlyEqual(result.b, truth.b, rel_tol, "B");
  ok &= NearlyEqual(result.tau_c, truth.tau_c, rel_tol, "tau_c");
  if (pitch_axis) {
    ok &= NearlyEqual(result.mgl, truth.mgl, rel_tol, "mgl");
  }
  if (result.residual_ratio > 0.05f) {
    std::printf("FAIL: residual_ratio=%.4f > 0.05\n", result.residual_ratio);
    ok = false;
  }
  return ok;
}

int Run() {
  bool ok = true;

  // ---- yaw 轴：无噪声 ----
  {
#ifdef SYSID_MUTATE_J
    const Truth TRUTH{0.04373488 * 1.5, 0.02, 0.03, 0.0};  // 变异：J 偏移 50%
#else
    const Truth TRUTH{0.04373488, 0.02, 0.03, 0.0};
#endif
    const Truth EXPECTED{0.04373488, 0.02, 0.03, 0.0};
    AxisResultOut result{};
    if (!RunIdentification(MakeConfig(IdentifyAxis::YAW_ONLY), false, TRUTH,
                           0.0, result)) {
      return 1;
    }
    ok &= CheckTruth(result, EXPECTED, 0.02, false);
    std::printf("yaw noiseless: J=%.6f B=%.5f tau_c=%.5f ratio=%.4f n=%u\n",
                result.j, result.b, result.tau_c, result.residual_ratio,
                result.update_count);
  }

  // ---- yaw 轴：含噪声 ----
  {
    const Truth TRUTH{0.04373488, 0.02, 0.03, 0.0};
    AxisResultOut result{};
    if (!RunIdentification(MakeConfig(IdentifyAxis::YAW_ONLY), false, TRUTH,
                           0.02, result)) {
      return 1;
    }
    ok &= CheckTruth(result, TRUTH, 0.10, false);
    std::printf("yaw noisy:     J=%.6f B=%.5f tau_c=%.5f ratio=%.4f\n",
                result.j, result.b, result.tau_c, result.residual_ratio);
  }

  // ---- pitch 轴：无噪声，含重力项 ----
  // 真值选取保证激励下 |omega| 不触 omega_abort_pit：
  // omega_ss ≈ (0.2 - 0.02 - 0.1*sin(theta))/0.1 ≈ 1.6 rad/s。
  {
    const Truth TRUTH{0.014, 0.1, 0.02, 0.1};
    AxisResultOut result{};
    if (!RunIdentification(MakeConfig(IdentifyAxis::PITCH_ONLY), true, TRUTH,
                           0.0, result)) {
      return 1;
    }
    ok &= CheckTruth(result, TRUTH, 0.02, true);
    std::printf(
        "pitch noiseless: J=%.6f B=%.5f tau_c=%.5f mgl=%.5f ratio=%.4f\n",
        result.j, result.b, result.tau_c, result.mgl, result.residual_ratio);
  }

  // ---- 中止路径：电机离线必须触发 MOTOR_OFFLINE ----
  {
    SystemIdentify sysid(MakeConfig(IdentifyAxis::YAW_ONLY));
    sysid.Begin();
    SystemIdentify::Sensors sensors{};
    sensors.rc_online = true;
    sensors.yaw_motor_online = false;  // 离线
    sensors.pit_motor_online = true;
    for (int i = 0; i < 500 && sysid.State() != IdentifyState::ABORTED; i++) {
      sysid.Update(sensors, static_cast<float>(DT_S));
    }
    if (sysid.State() != IdentifyState::ABORTED ||
        sysid.AbortReason() != IdentifyAbort::MOTOR_OFFLINE) {
      std::printf("FAIL: motor-offline abort path not taken\n");
      ok = false;
    }
  }

  // ---- 中止路径：超速必须触发 OMEGA_LIMIT ----
  {
    SystemIdentify sysid(MakeConfig(IdentifyAxis::YAW_ONLY));
    sysid.Begin();
    SystemIdentify::Sensors sensors{};
    sensors.rc_online = true;
    sensors.yaw_motor_online = true;
    sensors.pit_motor_online = true;
    sensors.yaw_omega_rad_s = 100.0f;  // 远超 omega_abort_yaw
    for (int i = 0; i < 500 && sysid.State() != IdentifyState::ABORTED; i++) {
      sysid.Update(sensors, static_cast<float>(DT_S));
    }
    if (sysid.AbortReason() != IdentifyAbort::OMEGA_LIMIT) {
      std::printf("FAIL: omega-limit abort path not taken\n");
      ok = false;
    }
  }

  if (ok) {
    std::printf("PASS: SystemIdentify simulation regression\n");
    return 0;
  }
  return 1;
}

}  // namespace

int main() { return Run(); }

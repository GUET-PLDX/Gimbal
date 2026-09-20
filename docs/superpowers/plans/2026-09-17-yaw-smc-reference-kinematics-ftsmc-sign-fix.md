# Yaw SMC Reference Kinematics and FTSMC Sign Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修正 `YawSmc` 的目标运动学量纲和 FTSMC 负误差导数符号，同时保持外部接口与输出保护行为不变。

**Architecture:** `YawSmc::Calculate()` 直接消费 `Reference` 中的角度、角速度和角加速度，并使用数学正确的 FTSMC 分数次幂导数。测试先用解析值和奇对称性质暴露旧行为，再最小修改控制器，最后同步文档并完成模块与固件回归验证。

**Tech Stack:** C++20、LibXR、header-only Gimbal module、PowerShell 7、Bash host regression、CMake/Clang 或 GCC ARM 工具链。

---

## 文件结构

- Modify: `Modules/Gimbal/YawSmc.hpp`：修正参考状态数据流、FTSMC 导数项并删除目标历史状态。
- Modify: `Modules/Gimbal/tests/yaw_smc_test_support.hpp`：将独立 oracle 改为 SI 参考状态和正确数学公式。
- Modify: `Modules/Gimbal/tests/yaw_smc_test.cpp`：增加先失败的参考运动学、周期独立性和 FTSMC 对称性测试；删除旧目标历史测试。
- Modify: `Modules/Gimbal/README.md`：更新数学约定、参考状态来源和迁移说明。
- No change: `User/RobotConfig/sentry_gimbal.yaml`：本计划不混入实车参数调整。
- No change: `Modules/Gimbal/Gimbal.hpp`：现有调用方已经提供完整 `theta_ref/omega_ref/alpha_ref`。

### Task 1: 用失败测试锁定参考运动学契约

**Files:**
- Modify: `Modules/Gimbal/tests/yaw_smc_test.cpp`

- [ ] **Step 1: 将旧目标历史测试替换为直接参考状态测试**

删除 `test_wrapped_target_delta_is_short_arc()` 和 `test_source_target_history()`，添加以下测试：

```cpp
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
  CHECK_NEAR(output.tau_pre_limit_nm,
             EXPECTED_TAU_FF_NM + EXPECTED_TAU_SMC_NM, 1.0e-6f);
}
```

- [ ] **Step 2: 添加关闭 slew 后核心控制律与 `dt` 无关的测试**

```cpp
static void test_core_law_is_independent_of_valid_dt() {
  auto cfg = base_yaw_smc_config();
  cfg.ftsmc_enable = false;
  cfg.torque_slew_enable = false;

  YawSmc fast_controller;
  YawSmc slow_controller;
  fast_controller.Reset(0.0f, 0.0f, 0.0f);
  slow_controller.Reset(0.0f, 0.0f, 0.0f);

  const auto fast = calculate_once(fast_controller, cfg, 0.1f, 0.4f, 3.0f,
                                   0.2f, 0.7f, 0.001f);
  const auto slow = calculate_once(slow_controller, cfg, 0.1f, 0.4f, 3.0f,
                                   0.2f, 0.7f, 0.010f);

  CHECK(fast.valid && slow.valid);
  CHECK_NEAR(fast.e_omega_rad_s, slow.e_omega_rad_s, 1.0e-7f);
  CHECK_NEAR(fast.tau_ff_alpha_nm, slow.tau_ff_alpha_nm, 1.0e-7f);
  CHECK_NEAR(fast.tau_pre_limit_nm, slow.tau_pre_limit_nm, 1.0e-7f);
}
```

- [ ] **Step 3: 在 `main()` 注册新测试**

```cpp
test_reference_kinematics_are_consumed_directly();
test_core_law_is_independent_of_valid_dt();
```

删除：

```cpp
test_wrapped_target_delta_is_short_arc();
test_source_target_history();
```

- [ ] **Step 4: 运行主机测试并确认按预期失败**

Run from Git Bash or WSL:

```bash
bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
```

Expected: FAIL；`e_omega_rad_s` 仍来自旧目标历史，`tau_ff_alpha_nm` 仍不等于 `J * alpha_ref`。

- [ ] **Step 5: 提交测试基线**

```bash
git -C Modules/Gimbal add tests/yaw_smc_test.cpp
git -C Modules/Gimbal commit -m "test: define yaw SMC reference kinematics"
```

### Task 2: 修正参考运动学实现

**Files:**
- Modify: `Modules/Gimbal/YawSmc.hpp`
- Modify: `Modules/Gimbal/tests/yaw_smc_test_support.hpp`

- [ ] **Step 1: 更新独立 oracle 的参考运动学**

将 `smc_code_oracle()` 中目标历史参数和差分逻辑删除，改为：

```cpp
inline SmcCodeOracle smc_code_oracle(const YawSmc::Config& config,
                                     const YawSmc::Reference& reference,
                                     const YawSmc::Feedback& feedback) {
  SmcCodeOracle oracle{};
  oracle.e_theta_rad =
      LibXR::CycleValue<float>(feedback.theta_rad) - reference.theta_rad;
  oracle.e_omega_rad_s = feedback.omega_rad_s - reference.omega_rad_s;
  oracle.in_deadband =
      std::fabs(oracle.e_theta_rad) < config.error_deadband_rad;
  if (oracle.in_deadband) {
    return oracle;
  }

  oracle.tau_ff_alpha_nm = config.j_kg_m2 * reference.alpha_rad_s2;
```

保留 oracle 的后续限幅前核心计算，FTSMC 符号将在 Task 4 单独修正。

- [ ] **Step 2: 修改 `YawSmc::Reset()`，停止初始化目标历史**

```cpp
void Reset(float theta_rad, float omega_rad_s,
           float previous_applied_torque_nm) {
  UNUSED(theta_rad);
  UNUSED(omega_rad_s);
  last_applied_torque_nm_ = previous_applied_torque_nm;
  slew_anchor_torque_nm_ = previous_applied_torque_nm;
  previous_torque_slew_enable_ = false;
}
```

- [ ] **Step 3: 在 `Calculate()` 中直接使用参考角速度与角加速度**

用以下逻辑替换 `TARGET_DELTA_RAD`、`TARGET_DDOT_CODE` 和目标历史更新：

```cpp
output.e_theta_rad =
    LibXR::CycleValue<float>(feedback.theta_rad) - reference.theta_rad;
output.e_omega_rad_s = feedback.omega_rad_s - reference.omega_rad_s;
```

死区判断之后使用：

```cpp
output.tau_ff_alpha_nm = config.j_kg_m2 * reference.alpha_rad_s2;
```

函数成功返回前不再更新目标历史。

- [ ] **Step 4: 删除目标历史成员**

删除：

```cpp
float target_last_rad_{};
float target_dot_rad_s_{};
```

- [ ] **Step 5: 运行主机测试并确认参考运动学测试通过**

```bash
bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
```

Expected: PASS；若旧 oracle 断言仍依赖目标历史，则只修改对应期望，不改保护逻辑。

- [ ] **Step 6: 提交参考运动学修复**

```bash
git -C Modules/Gimbal add YawSmc.hpp tests/yaw_smc_test_support.hpp \
  tests/yaw_smc_test.cpp
git -C Modules/Gimbal commit -m "fix: consume yaw SMC reference kinematics"
```

### Task 3: 用失败测试锁定 FTSMC 正确符号

**Files:**
- Modify: `Modules/Gimbal/tests/yaw_smc_test.cpp`

- [ ] **Step 1: 添加 FTSMC 奇对称测试**

```cpp
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

  const auto positive = calculate_once(positive_controller, cfg, 0.0f, 0.0f,
                                       0.0f, 0.1f, 0.2f);
  const auto negative = calculate_once(negative_controller, cfg, 0.0f, 0.0f,
                                       0.0f, -0.1f, -0.2f);

  CHECK(positive.valid && negative.valid);
  CHECK(positive.used_ftsmc && negative.used_ftsmc);
  CHECK_NEAR(negative.s, -positive.s, 1.0e-6f);
  CHECK_NEAR(negative.tau_smc_nm, -positive.tau_smc_nm, 1.0e-6f);
}
```

- [ ] **Step 2: 添加负误差解析公式测试**

```cpp
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
  const float S = E_OMEGA_RAD_S +
                  cfg.c * std::copysign(
                              std::pow(std::fabs(E_THETA_RAD), R),
                              E_THETA_RAD);
  const float SURFACE_DOT_TERM =
      cfg.c * R * std::pow(std::fabs(E_THETA_RAD), R - 1.0f) *
      E_OMEGA_RAD_S;
  const float EXPECTED_TAU_SMC_NM =
      cfg.j_kg_m2 * (-SURFACE_DOT_TERM - cfg.epsilon * sat(S / cfg.sat_boundary) -
                     cfg.k * S);

  CHECK(output.valid && output.used_ftsmc);
  CHECK_NEAR(output.tau_smc_nm, EXPECTED_TAU_SMC_NM, 1.0e-5f);
}
```

- [ ] **Step 3: 在 `main()` 注册两个测试**

```cpp
test_ftsmc_feedback_is_odd_symmetric();
test_ftsmc_negative_error_uses_unsigned_power_derivative();
```

- [ ] **Step 4: 运行测试并确认旧实现失败**

```bash
bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
```

Expected: FAIL；负误差的 `tau_smc_nm` 不满足奇对称，且与解析公式不符。

- [ ] **Step 5: 提交失败测试**

```bash
git -C Modules/Gimbal add tests/yaw_smc_test.cpp
git -C Modules/Gimbal commit -m "test: cover negative-error FTSMC dynamics"
```

### Task 4: 修正 FTSMC 分数次幂导数

**Files:**
- Modify: `Modules/Gimbal/YawSmc.hpp`
- Modify: `Modules/Gimbal/tests/yaw_smc_test_support.hpp`

- [ ] **Step 1: 修正生产实现的 `surface_dot_term`**

将 FTSMC 分支改为：

```cpp
if (USE_FTSMC) {
  const float R = config.q / config.p;
  output.s =
      output.e_omega_rad_s + config.c * SigPow(output.e_theta_rad, R);
  surface_dot_term =
      config.c * R *
      std::pow(ABS_E_THETA_RAD, R - 1.0f) * output.e_omega_rad_s;
} else {
  output.s = output.e_omega_rad_s + config.c * output.e_theta_rad;
  surface_dot_term = config.c * output.e_omega_rad_s;
}
```

同时更新邻近注释，明确 `d(sig^r(e))/dt = r*abs(e)^(r-1)*e_dot`。

- [ ] **Step 2: 独立修正测试 oracle**

将 oracle 的 FTSMC 导数项改为：

```cpp
surface_dot_term =
    config.c * R *
    std::pow(ABS_E_THETA_RAD, R - 1.0f) * oracle.e_omega_rad_s;
```

不要调用 `YawSmc::SigPow()` 或其他私有帮助函数，保持 oracle 与生产实现相互独立。

- [ ] **Step 3: 运行主机测试**

```bash
bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
```

Expected: PASS，输出无 sanitizer 或浮点非有限错误。

- [ ] **Step 4: 使用 sanitizer 再运行一次**

```bash
SANITIZE=1 bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
```

Expected: PASS，无 AddressSanitizer 或 UndefinedBehaviorSanitizer 报告。

- [ ] **Step 5: 提交公式修复**

```bash
git -C Modules/Gimbal add YawSmc.hpp tests/yaw_smc_test_support.hpp \
  tests/yaw_smc_test.cpp
git -C Modules/Gimbal commit -m "fix: correct negative-error FTSMC dynamics"
```

### Task 5: 同步算法文档

**Files:**
- Modify: `Modules/Gimbal/README.md`

- [ ] **Step 1: 更新算法来源说明**

将“严格按 `SMC_Tick()` 计算顺序移植”改为：

```markdown
`YawSmc` 基于参考工程的 SMC/FTSMC 结构实现，并使用 SI 单位和力矩接口。
当前实现直接消费调用方提供的目标角度、角速度和角加速度，不使用目标角度的无周期差分。
```

- [ ] **Step 2: 更新参考状态与控制律**

文档必须包含：

```text
e = wrap(theta - theta_ref)
e_dot = omega - omega_ref
tau_ff_alpha = J * alpha_ref
```

FTSMC 导数项必须写为：

```text
surface_dot_term = c * (q/p) * abs(e)^(q/p - 1) * e_dot
```

- [ ] **Step 3: 删除旧目标历史语义**

删除 README 中关于 `target_last`、`target_dot`、`target_ddot` 不按 `dt` 归一化，以及 `Reference::omega_rad_s/alpha_rad_s2` 不参与计算的描述。

- [ ] **Step 4: 增加迁移说明**

```markdown
修复后，移动目标的速度误差和角加速度前馈将直接使用调用方参考状态；FTSMC
负误差方向的反馈项也会与正方向保持数学对称。升级后应重新执行正负角度阶跃、
匀速目标和加速目标测试，再进行实车参数整定。
```

- [ ] **Step 5: 提交文档**

```bash
git -C Modules/Gimbal add README.md
git -C Modules/Gimbal commit -m "docs: clarify yaw SMC reference-state contract"
```

### Task 6: 完整验证与交付检查

**Files:**
- Verify only: `Modules/Gimbal/**`
- Verify only: `User/RobotConfig/sentry_gimbal.yaml`

- [ ] **Step 1: 运行 SMC 主机测试**

```bash
bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
```

Expected: exit code `0`，无失败断言。

- [ ] **Step 2: 运行 Gimbal 核心静态回归**

```bash
bash Modules/Gimbal/tests/gimbal_core_static_regression.sh
```

Expected: exit code `0`。

- [ ] **Step 3: 运行格式检查**

```powershell
& 'C:\Program Files\PowerShell\7\pwsh.exe' -NoProfile -Command `
  'pwsh tools/format_code.ps1 --check'
```

Expected: exit code `0`，无格式差异。

- [ ] **Step 4: 编译 Gimbal 固件**

```powershell
& 'C:\Program Files\PowerShell\7\pwsh.exe' -NoProfile -Command `
  'pwsh tools/buildgimbal.ps1 --skip-format'
```

Expected: `sentry_gimbal` 编译和链接成功，`-Werror` 下无警告。

- [ ] **Step 5: 编译 Chassis 固件以验证共享模块集成**

```powershell
& 'C:\Program Files\PowerShell\7\pwsh.exe' -NoProfile -Command `
  'pwsh tools/buildchassis.ps1 --skip-format'
```

Expected: `sentry_chassis` 编译和链接成功。

- [ ] **Step 6: 检查差异范围**

```powershell
& 'C:\Program Files\PowerShell\7\pwsh.exe' -NoProfile -Command `
  'git -C Modules/Gimbal diff --check; git -C Modules/Gimbal status --short; git diff -- User/RobotConfig/sentry_gimbal.yaml'
```

Expected:

- `diff --check` 无输出。
- 只有计划中的 Gimbal 文件发生变化。
- `sentry_gimbal.yaml` 无算法修复导致的参数改动。

- [ ] **Step 7: 记录实车验证清单但不在本代码任务中改参**

依次执行并记录：

```text
1. 静止目标保持：确认无新增静止漂移。
2. +5 deg / -5 deg 对称阶跃：比较上升时间、超调和峰值力矩。
3. +15 deg / -15 deg 对称阶跃：确认 FTSMC 两侧响应对称。
4. 匀速目标：确认 omega_ref 降低跟踪滞后。
5. 加速目标：确认 alpha_ref 前馈方向和幅值正确。
6. 观察 soft_limit_active 与 slew_limit_active，避免把饱和误判为增益问题。
```

若实车需要调参，另开独立提交修改 `User/RobotConfig/sentry_gimbal.yaml`。

## 自检结果

- 规格覆盖：两个已确认问题分别由 Task 1-2 和 Task 3-4 覆盖。
- 接口一致：计划不改变 `Reference`、`Calculate()`、`Reset()` 或 YAML 字段。
- 测试独立：关键正确性由解析公式与奇对称性质验证，不只依赖复制实现的 oracle。
- 范围约束：算法修复、文档同步和实车参数调整彼此分离。
- 占位符检查：所有步骤均包含明确实现内容和预期结果。

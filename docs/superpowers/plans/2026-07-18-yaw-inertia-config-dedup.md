# Yaw Inertia Configuration Deduplication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the original Gimbal `j_yaw` parameter the only configured Yaw inertia used by both Legacy Yaw and AI Yaw LQR/ESO.

**Architecture:** Remove plant inertia from `YawLqrEso::Config` and pass it explicitly to `ValidateConfig()` and `Calculate()`. `Gimbal::SolveAiYaw()` supplies its existing `j_yaw_`; the controller continues to use that value for acceleration feedforward, ESO input gain, and the ESO viscous model without changing routing or numerical behavior.

**Tech Stack:** C++17 firmware, C++20 host tests, LibXR/xrobot module manifest, Python/PyYAML contract regression, Bash/ripgrep static integration regression, CMake/STM32 cross-build.

## Global Constraints

- Keep Legacy Pitch and Yaw formulas unchanged.
- Keep AI Yaw activation, reset, invalid-output fallback, and torque submission unchanged.
- Do not merge `yaw_k` with `b_nms_rad`, PID gains with LQR gains, or PID output limits with AI torque limits.
- Preserve the current numerical inertia value: `j_yaw: 0.03` remains the single configured value.
- Preserve unrelated changes in `/home/sb/PLDX_Template/User/RobotConfig/sentry_gimbal.yaml`.
- Do not stage or commit `/home/sb/PLDX_Template/Modules/Gimbal/tests/__pycache__/`.

## File Map

- Modify `YawLqrEso.hpp`: remove inertia from controller tuning and accept explicit plant inertia.
- Modify `Gimbal.hpp`: remove the duplicate manifest value and pass `j_yaw_` to the AI controller.
- Modify `tests/gimbal_config_order_regression.py`: enforce the shortened aggregate contract.
- Modify `tests/ai_yaw_integration_regression.sh`: enforce single-source inertia wiring.
- Modify `tests/yaw_lqr_eso_test_support.hpp`: provide a host-test plant inertia constant, not a config field.
- Modify `tests/yaw_lqr_eso_test.cpp`: migrate API calls and retain invalid-inertia coverage.
- Modify `tests/yaw_lqr_eso_simulation_test.cpp`: pass controller-model inertia explicitly and use it for ESO metrics.
- Modify `/home/sb/PLDX_Template/User/RobotConfig/sentry_gimbal.yaml`: remove the duplicate target configuration key.
- Regenerate `/home/sb/PLDX_Template/User/xrobot_main.hpp`: verify xrobot aggregate generation; do not commit generated churn unless the parent repository intentionally reviews it.

---

### Task 1: Atomically Migrate the Gimbal Module to Single-Source Yaw Inertia

**Files:**
- Modify: `tests/gimbal_config_order_regression.py`
- Modify: `tests/ai_yaw_integration_regression.sh`
- Modify: `tests/yaw_lqr_eso_test_support.hpp`
- Modify: `tests/yaw_lqr_eso_test.cpp`
- Modify: `tests/yaw_lqr_eso_simulation_test.cpp`
- Modify: `YawLqrEso.hpp`
- Modify: `Gimbal.hpp`

**Interfaces:**
- Consumes: existing Gimbal constructor parameter and member `float j_yaw_`.
- Produces: `static bool YawLqrEso::ValidateConfig(const Config&, float j_kg_m2)`.
- Produces: `YawLqrEso::Output Calculate(const Config&, const Reference&, const Feedback&, float dt_s, float j_kg_m2)`.
- Produces: a `yaw_lqr_eso` manifest mapping whose first field is `b_nms_rad` and which contains no `j_kg_m2`.

- [ ] **Step 1: Write failing configuration and integration regressions**

Remove `"j_kg_m2"` from `EXPECTED_FIELDS` and `EXPECTED_DEFAULTS` in `tests/gimbal_config_order_regression.py`, then add after `yaw_manifest` is loaded:

```python
if "j_kg_m2" in yaw_manifest:
    raise SystemExit("Yaw inertia must come from the Gimbal j_yaw parameter")
```

Add the corresponding target-YAML check after `yaw_yaml` is loaded:

```python
if "j_kg_m2" in yaw_yaml:
    raise SystemExit("target YAML must not duplicate the Gimbal j_yaw parameter")
```

Add these checks to `tests/ai_yaw_integration_regression.sh` before the existing route assertions:

```bash
forbid_file "${ALGORITHM_HEADER}" 'float j_kg_m2\{\}' \
  'duplicate Yaw inertia in controller Config'
forbid_file "${HEADER}" '^[[:space:]]+j_kg_m2:' \
  'duplicate Yaw inertia in the Gimbal manifest'
need_multiline \
  'yaw_lqr_eso_\.Calculate\(.*dt_,\s*j_yaw_\s*\)' \
  'AI Yaw calculation receives the original Gimbal inertia'
```

- [ ] **Step 2: Migrate host tests first so the old controller API fails to compile**

In `tests/yaw_lqr_eso_test_support.hpp`, remove `.j_kg_m2 = 0.03f` from `base_yaw_config()` and add:

```cpp
inline constexpr float TEST_YAW_J_KG_M2 = 0.03f;
```

In `tests/yaw_lqr_eso_test.cpp`, append `TEST_YAW_J_KG_M2` to every `controller.Calculate(...)` call. Change configuration validation to:

```cpp
static void test_config_validation() {
  auto cfg = base_yaw_config();
  CHECK(YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2));
  CHECK(!YawLqrEso::ValidateConfig(cfg, 0.0f));
  CHECK(!YawLqrEso::ValidateConfig(
      cfg, std::numeric_limits<float>::quiet_NaN()));
  cfg.k_theta = -1.0f;
  CHECK(!YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2));
  cfg = base_yaw_config();
  cfg.torque_bias_enable = true;
  cfg.tau_meas_lpf_alpha = 1.1f;
  CHECK(!YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2));
  cfg = base_yaw_config();
  cfg.k_omega = std::numeric_limits<float>::quiet_NaN();
  CHECK(!YawLqrEso::ValidateConfig(cfg, TEST_YAW_J_KG_M2));
}
```

Replace the observer expectation's duplicate config access:

```cpp
const float B0 = 1.0f / TEST_YAW_J_KG_M2;
```

In `tests/yaw_lqr_eso_simulation_test.cpp`, append `TEST_YAW_J_KG_M2` to the adapter's controller call:

```cpp
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
    static_cast<float>(dt), TEST_YAW_J_KG_M2);
```

Compute the ESO metric with the same test model value:

```cpp
const double ESTIMATED_DISTURBANCE_TORQUE =
    static_cast<double>(TEST_YAW_J_KG_M2) * OUTPUT.z3;
```

- [ ] **Step 3: Run the focused checks and verify the expected RED state**

Run:

```bash
python3 tests/gimbal_config_order_regression.py \
  --header Gimbal.hpp --algorithm YawLqrEso.hpp --header-only
bash tests/ai_yaw_integration_regression.sh Gimbal.hpp
bash tests/yaw_lqr_eso_host_regression.sh
```

Expected failures:

- Configuration regression: `Config order mismatch` because production `Config` still contains `j_kg_m2`.
- Integration regression: `duplicate Yaw inertia in controller Config` or `duplicate Yaw inertia in the Gimbal manifest`.
- Host build: no matching `ValidateConfig`/`Calculate` overload accepting the explicit inertia argument.

- [ ] **Step 4: Remove inertia from `YawLqrEso::Config` and migrate the algorithm API**

Delete this member from `YawLqrEso::Config`:

```cpp
float j_kg_m2{};
```

Change validation to accept and validate the plant parameter independently:

```cpp
static bool ValidateConfig(const Config& config, float j_kg_m2) {
  if (!AllConfigFloatsFinite(config) || !std::isfinite(j_kg_m2) ||
      j_kg_m2 <= MIN_J_KG_M2 || config.b_nms_rad < 0.0f ||
      config.k_theta < 0.0f || config.k_omega < 0.0f ||
      config.theta_deadband_rad < 0.0f) {
    return false;
  }
```

Change the calculation signature and validation call:

```cpp
Output Calculate(const Config& config, const Reference& reference,
                 const Feedback& feedback, float dt_s, float j_kg_m2) {
  Output output{};
  if (!ValidateConfig(config, j_kg_m2) || !feedback.valid ||
```

Use the explicit argument at every plant-model site:

```cpp
output.tau_ff_alpha_nm = j_kg_m2 * reference.alpha_rad_s2;
const float B0 = 1.0f / j_kg_m2;
const float Z2_DOT = -(config.b_nms_rad / j_kg_m2) * z2_ +
                     B0 * last_applied_torque_nm_ + z3_ +
                     BETA2 * OBSERVER_ERROR_RAD;
```

The ESO compensation block must likewise use:

```cpp
const float B0 = 1.0f / j_kg_m2;
```

Remove `std::isfinite(config.j_kg_m2) &&` from `AllConfigFloatsFinite()`; do not change validation of any controller tuning field.

- [ ] **Step 5: Remove the duplicate manifest field and wire `j_yaw_` into AI Yaw**

Delete this manifest line from `Gimbal.hpp`:

```yaml
      j_kg_m2: 0.03
```

Append the original Gimbal member after `dt_` in `SolveAiYaw()`:

```cpp
    const auto YAW_LQR_ESO_OUTPUT = yaw_lqr_eso_.Calculate(
        yaw_lqr_eso_config_,
        {.theta_rad = cmd_data_.yaw,
         .omega_rad_s = cmd_data_.yaw_dot,
         .alpha_rad_s2 = cmd_data_.yaw_ddot},
        {.theta_rad = euler_.Yaw(),
         .omega_rad_s = gyro_data_.z(),
         .tau_meas_nm = motor_yaw_feedback_.torque,
         .valid = motor_feedback_online_,
         .torque_measurement_valid = std::isfinite(motor_yaw_feedback_.torque)},
        dt_, j_yaw_);
```

- [ ] **Step 6: Run focused GREEN verification**

Run:

```bash
python3 tests/gimbal_config_order_regression.py \
  --header Gimbal.hpp --algorithm YawLqrEso.hpp --header-only
bash tests/ai_yaw_integration_regression.sh Gimbal.hpp
bash tests/yaw_lqr_eso_host_regression.sh
```

Expected `PASS: Gimbal config order regression`, `PASS: AI Yaw direct-routing integration regression`, and host regression exit `0` after compiling all `yaw_*_test.cpp` files with warnings as errors.

- [ ] **Step 7: Run the remaining module regression suite**

Run:

```bash
python3 tests/parse_cmd_behavior_regression.py --header Gimbal.hpp
python3 tests/parse_cmd_behavior_regression.py \
  --header Gimbal.hpp --negative-checks
bash tests/gimbal_core_static_regression.sh Gimbal.hpp
python3 tests/selected_feature_removal_regression.py --header Gimbal.hpp
python3 tests/workflow_permissions_regression.py \
  --workflow .github/workflows/build.yml
git diff --check
```

Expected: every command exits `0`; the ParseCMD negative run ends with `PASS: 16 ParseCMD negative behavior checks`.

- [ ] **Step 8: Commit the atomic module migration**

Review `git diff` and ensure the design document commit remains separate and `tests/__pycache__/` is untracked. Then run:

```bash
git add Gimbal.hpp YawLqrEso.hpp \
  tests/gimbal_config_order_regression.py \
  tests/ai_yaw_integration_regression.sh \
  tests/yaw_lqr_eso_test_support.hpp \
  tests/yaw_lqr_eso_test.cpp \
  tests/yaw_lqr_eso_simulation_test.cpp
git commit -m "refactor(gimbal): reuse yaw inertia in AI controller"
```

---

### Task 2: Update the Parent Robot Configuration and Verify Firmware Integration

**Files:**
- Modify: `/home/sb/PLDX_Template/User/RobotConfig/sentry_gimbal.yaml`
- Regenerate for verification: `/home/sb/PLDX_Template/User/xrobot_main.hpp`

**Interfaces:**
- Consumes: the shortened `YawLqrEso::Config` aggregate from Task 1.
- Produces: target YAML with `j_yaw: 0.03` as the only Yaw inertia source.
- Produces: generated Gimbal construction matching the shortened manifest order.

- [ ] **Step 1: Remove only the duplicate target YAML key**

Delete exactly this line from the Gimbal `yaw_lqr_eso` mapping:

```yaml
      j_kg_m2: 0.03
```

Keep the existing original parameter unchanged:

```yaml
    j_yaw: 0.03
```

Do not alter or revert the existing WsProtocol, SharedTopic, Referee, or IMU topic edits in the same working tree.

- [ ] **Step 2: Build the sentry Gimbal firmware to regenerate and compile the aggregate**

From `/home/sb/PLDX_Template`, run:

```bash
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/sentry_gimbal
```

Expected: xrobot generation succeeds, the STM32 build exits `0`, and the final output contains `Done.` with FLASH and RAM usage.

- [ ] **Step 3: Verify the target YAML and generated constructor aggregate**

Run:

```bash
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml \
  --generated User/xrobot_main.hpp
```

Expected: `PASS: Gimbal config order regression`.

- [ ] **Step 4: Audit both repositories before handoff**

Run:

```bash
git -C Modules/Gimbal status --short --branch
git -C Modules/Gimbal log -2 --oneline
git status --short -- User/RobotConfig/sentry_gimbal.yaml User/xrobot_main.hpp
rg -n 'j_kg_m2' Modules/Gimbal/Gimbal.hpp \
  Modules/Gimbal/YawLqrEso.hpp \
  User/RobotConfig/sentry_gimbal.yaml
```

Expected:

- The module branch contains the design commit and one implementation commit.
- `tests/__pycache__/` remains untracked and unstaged.
- The parent YAML remains modified alongside the user's pre-existing changes.
- The final `rg` finds no externally configured or controller-config member named `j_kg_m2`; occurrences are limited to explicit API parameter names if that descriptive name is retained.
- Do not commit the parent repository automatically because the target YAML already contains unrelated user changes; report its exact diff for separate review.

# Gimbal Snapshot and ParseCMD Deduplication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove four unnecessary snapshot members, then simplify `Gimbal::ParseCMD()` without changing any command behavior.

**Architecture:** Keep all target generation inside `Gimbal`, replace per-loop snapshots with local constants, and organize `ParseCMD()` into fact capture, Pitch update, and legacy Yaw update phases. Lock every existing branch with static characterization checks before rewriting the function.

**Tech Stack:** C++17/20 header-only module code, LibXR topics and PID, Python 3 static regression scripts, Bash regression runners, clang-format 21.1.8, STM32 `tools/build.sh` pipeline.

## Global Constraints

- Preserve operator, low-sensitivity, AI, automatic patrol, and non-AI automatic command semantics exactly.
- Preserve Pitch-before-Yaw processing order.
- Preserve motor-feedback protection, rotor feedforward, target derivative feedforward, and `YawLqrEso` behavior.
- Do not change the public constructor, manifest, YAML fields, topics, or motor command types.
- Follow project naming: all `const` local identifiers use `UPPER_CASE`.
- Format only `Modules/Gimbal/Gimbal.hpp` with clang-format 21.1.8.

---

## File Map

- `Gimbal.hpp`: remove snapshot state and later restructure `ParseCMD()`.
- `tests/selected_feature_removal_regression.py`: enforce removal of the four snapshot member names and direct configuration use.
- `tests/ai_yaw_integration_regression.sh`: update AI selection assertions from members to local constants.
- `tests/parse_cmd_behavior_regression.py`: characterize all current `ParseCMD()` branches before deduplication.
- `docs/superpowers/specs/2026-07-17-gimbal-parsecmd-dedup-design.md`: approved behavioral source of truth; no modification planned.

### Task 1: Remove Snapshot Members

**Files:**
- Modify: `Gimbal.hpp:312-320,488-497,601-627`
- Modify: `tests/selected_feature_removal_regression.py`
- Modify: `tests/ai_yaw_integration_regression.sh`

**Interfaces:**
- Consumes: `CMD::GetCtrlMode()`, `CMD::GetAIGimbalStatus()`, `YawLqrEso::Calculate()`.
- Produces: unchanged `void ParseCMD()` and `void SolveAiYaw()` methods with no snapshot members.

- [ ] **Step 1: Add failing snapshot-removal assertions**

Add these forbidden entries to `tests/selected_feature_removal_regression.py`:

```python
    "control-mode snapshot member": "ctrl_mode_snapshot_",
    "AI-status snapshot member": "ai_gimbal_status_snapshot_",
    "AI-config snapshot member": "yaw_lqr_eso_config_snapshot_",
    "AI-output snapshot member": "yaw_lqr_eso_output_",
```

Add these required regular expressions:

```python
    "local control mode": r"const\s+auto\s+CTRL_MODE\s*=\s*cmd_\.GetCtrlMode\(\)",
    "local AI status": r"const\s+bool\s+AI_GIMBAL_ACTIVE\s*=\s*cmd_\.GetAIGimbalStatus\(\)",
    "direct AI config": r"yaw_lqr_eso_\.Calculate\(\s*yaw_lqr_eso_config_",
    "local AI output": r"const\s+auto\s+YAW_LQR_ESO_OUTPUT\s*=\s*yaw_lqr_eso_\.Calculate",
```

- [ ] **Step 2: Run the new regression and verify RED**

Run:

```bash
python3 tests/selected_feature_removal_regression.py --header Gimbal.hpp
```

Expected: exit 1 with `forbidden: control-mode snapshot member`.

- [ ] **Step 3: Replace ParseCMD snapshots with local constants**

Replace the beginning of `ParseCMD()` with:

```cpp
const auto CTRL_MODE = cmd_.GetCtrlMode();
const bool AI_GIMBAL_ACTIVE = cmd_.GetAIGimbalStatus();
const bool AI_YAW_ACTIVE =
    CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && AI_GIMBAL_ACTIVE;
ai_yaw_active_ = AI_YAW_ACTIVE;
```

Within the existing function, replace `ctrl_mode_snapshot_` with `CTRL_MODE` and `ai_gimbal_status_snapshot_` with `AI_GIMBAL_ACTIVE`. Delete the assignment to `yaw_lqr_eso_config_snapshot_`.

- [ ] **Step 4: Localize the AI output and use the immutable config directly**

In `SolveAiYaw()`, replace the calculation and result reads with:

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
    dt_);
if (!YAW_LQR_ESO_OUTPUT.valid ||
    !std::isfinite(YAW_LQR_ESO_OUTPUT.tau_cmd_nm)) {
  yaw_output_ = 0.0f;
  yaw_lqr_eso_reset_pending_ = true;
  return;
}
yaw_lqr_eso_reset_pending_ = false;
yaw_output_ = YAW_LQR_ESO_OUTPUT.tau_cmd_nm;
```

Delete the four member declarations from the private member block.

- [ ] **Step 5: Update the AI integration assertion**

In `tests/ai_yaw_integration_regression.sh`, replace the snapshot-based AI selection expression with:

```bash
need_multiline \
  'const auto CTRL_MODE = cmd_\.GetCtrlMode\(\);\s*const bool AI_GIMBAL_ACTIVE = cmd_\.GetAIGimbalStatus\(\);\s*const bool AI_YAW_ACTIVE =\s*CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && AI_GIMBAL_ACTIVE;' \
  'local CMD-based AI selection'
```

- [ ] **Step 6: Format and verify GREEN**

Run:

```bash
clang-format -i Gimbal.hpp
python3 tests/selected_feature_removal_regression.py --header Gimbal.hpp
bash tests/gimbal_core_static_regression.sh Gimbal.hpp
bash tests/ai_yaw_integration_regression.sh Gimbal.hpp
python3 tests/gimbal_config_order_regression.py \
  --header Gimbal.hpp --algorithm YawLqrEso.hpp --header-only
bash tests/yaw_lqr_eso_host_regression.sh
clang-format --dry-run --Werror Gimbal.hpp
git diff --check
```

Expected: every command exits 0; the named scripts print `PASS`.

- [ ] **Step 7: Build the firmware target**

From `/home/sb/PLDX_Template`, run:

```bash
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/sentry_gimbal
```

Expected: exit 0, `DevC.elf`, `DevC.bin`, and `DevC.hex` generated.

- [ ] **Step 8: Commit the independently verified snapshot removal**

```bash
git add Gimbal.hpp \
  tests/selected_feature_removal_regression.py \
  tests/ai_yaw_integration_regression.sh
git commit -m "refactor(gimbal): remove transient AI snapshots"
```

### Task 2: Characterize ParseCMD Behavior

**Files:**
- Create: `tests/parse_cmd_behavior_regression.py`

**Interfaces:**
- Consumes: the source body of `void ParseCMD()` in `Gimbal.hpp`.
- Produces: a regression command that exits nonzero when any existing mode formula, sign, or ordering changes.

- [ ] **Step 1: Create the behavior regression**

Create `tests/parse_cmd_behavior_regression.py` with this implementation:

```python
import argparse
import pathlib
import re


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
args = parser.parse_args()
source = pathlib.Path(args.header).read_text()

match = re.search(
    r"\bvoid\s+ParseCMD\s*\(\s*\)\s*\{(.*?)\n\s*\}\n\n\s*/\*\*",
    source,
    re.S,
)
if match is None:
    raise SystemExit("ParseCMD body not found")
body = match.group(1)

required = {
    "operator low sensitivity": r"SET_MODE_LOW_SENSITIVITY.*?0\.1f",
    "AI absolute Pitch":
        r"AI_GIMBAL_ACTIVE.*?target_pit_cmd_\s*=\s*cmd_data_\.pit;.*?"
        r"target_pit_dot_\s*=\s*cmd_data_\.pit_dot;.*?"
        r"target_pit_ddot_\s*=\s*cmd_data_\.pit_ddot;",
    "patrol Pitch expression":
        r"SET_MODE_AUTOPATROL.*?patrol_range_.*?asin\(sin\(patrol_omega_",
    "patrol Yaw rate":
        r"SET_MODE_AUTOPATROL.*?target_yaw_cmd_\s*\+=\s*1\.0f\s*\*\s*dt_;",
    "negative automatic Yaw":
        r"YAW_OPERATOR_RATE\s*=\s*-cmd_data_\.yaw\s*\*\s*GIMBAL_MAX_SPEED;",
    "AI bypasses legacy Yaw":
        r"if\s*\(\s*!ai_yaw_active_\s*\)|"
        r"if\s*\(\s*AI_YAW_ACTIVE\s*\)\s*\{\s*return\s*;",
}
for description, pattern in required.items():
    if re.search(pattern, body, re.S) is None:
        raise SystemExit(f"missing: {description}")

pitch_position = body.find("target_pit_cmd_")
yaw_position = body.find("target_yaw_cmd_")
if pitch_position < 0 or yaw_position < 0 or pitch_position >= yaw_position:
    raise SystemExit("Pitch target processing must precede Yaw target processing")

print("PASS: ParseCMD behavior characterization")
```

- [ ] **Step 2: Run characterization against the current implementation**

Run:

```bash
python3 tests/parse_cmd_behavior_regression.py --header Gimbal.hpp
```

Expected: `PASS: ParseCMD behavior characterization`.

- [ ] **Step 3: Prove the test detects a semantic change**

Copy `Gimbal.hpp` to a temporary file, change the automatic Yaw rate from negative to positive using `sed`, and run the regression against the temporary file:

```bash
cp Gimbal.hpp /tmp/Gimbal-parsecmd-negative.hpp
sed -i 's/-cmd_data_\.yaw \* GIMBAL_MAX_SPEED/cmd_data_.yaw * GIMBAL_MAX_SPEED/' \
  /tmp/Gimbal-parsecmd-negative.hpp
python3 tests/parse_cmd_behavior_regression.py \
  --header /tmp/Gimbal-parsecmd-negative.hpp
```

Expected: exit 1 with `missing: negative automatic Yaw`.

- [ ] **Step 4: Commit the characterization test**

```bash
git add tests/parse_cmd_behavior_regression.py
git commit -m "test(gimbal): characterize ParseCMD branches"
```

### Task 3: Deduplicate ParseCMD In Place

**Files:**
- Modify: `Gimbal.hpp:312-378`
- Test: `tests/parse_cmd_behavior_regression.py`

**Interfaces:**
- Consumes: current `CMD::GimbalCMD`, `GimbalEvent`, `dt_`, and patrol state.
- Produces: unchanged `void ParseCMD()` with three explicit phases and no new members or helper methods.

- [ ] **Step 1: Replace ParseCMD with the phased implementation**

Use this complete method body:

```cpp
void ParseCMD() {
  const auto CTRL_MODE = cmd_.GetCtrlMode();
  const bool AI_GIMBAL_ACTIVE = cmd_.GetAIGimbalStatus();
  const bool OPERATOR_CONTROL = CTRL_MODE == CMD::Mode::CMD_OP_CTRL;
  const bool LOW_SENSITIVITY =
      current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY;
  const bool AUTOPATROL = current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL;
  const bool AI_YAW_ACTIVE =
      CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && AI_GIMBAL_ACTIVE;
  ai_yaw_active_ = AI_YAW_ACTIVE;

  if (AI_YAW_ACTIVE) {
    target_pit_cmd_ = cmd_data_.pit;
    target_pit_dot_ = cmd_data_.pit_dot;
    target_pit_ddot_ = cmd_data_.pit_ddot;
  } else if (!OPERATOR_CONTROL && AUTOPATROL) {
    target_pit_cmd_ -=
        patrol_range_ * (2 / M_PI) *
        asin(sin(patrol_omega_ *
                 (LibXR::Timebase::GetMilliseconds() - patrol_start_time))) /
        1000.0f;
    target_pit_dot_ = 0.0f;
    target_pit_ddot_ = 0.0f;
  } else {
    const float PITCH_SENSITIVITY =
        OPERATOR_CONTROL && LOW_SENSITIVITY ? 0.1f : 1.0f;
    const float PIT_OPERATOR_RATE =
        cmd_data_.pit * GIMBAL_MAX_SPEED * PITCH_SENSITIVITY;
    target_pit_cmd_ += PIT_OPERATOR_RATE * dt_;
    target_pit_dot_ = PIT_OPERATOR_RATE;
    target_pit_ddot_ = 0.0f;
  }

  if (AI_YAW_ACTIVE) {
    return;
  }

  if (OPERATOR_CONTROL) {
    const float YAW_SENSITIVITY = LOW_SENSITIVITY ? 0.1f : 1.0f;
    const float YAW_OPERATOR_RATE =
        cmd_data_.yaw * GIMBAL_MAX_SPEED * YAW_SENSITIVITY;
    target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
    target_yaw_dot_ = YAW_OPERATOR_RATE;
  } else if (AUTOPATROL) {
    target_yaw_cmd_ += 1.0f * dt_;
    target_yaw_dot_ = 1.0f;
  } else {
    const float YAW_OPERATOR_RATE = -cmd_data_.yaw * GIMBAL_MAX_SPEED;
    target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
    target_yaw_dot_ = YAW_OPERATOR_RATE;
  }
  target_yaw_ddot_ = 0.0f;
}
```

- [ ] **Step 2: Format and run focused regressions**

```bash
clang-format -i Gimbal.hpp
python3 tests/parse_cmd_behavior_regression.py --header Gimbal.hpp
python3 tests/selected_feature_removal_regression.py --header Gimbal.hpp
bash tests/gimbal_core_static_regression.sh Gimbal.hpp
bash tests/ai_yaw_integration_regression.sh Gimbal.hpp
```

Expected: all commands exit 0 and print their `PASS` messages.

- [ ] **Step 3: Run complete module and firmware verification**

```bash
python3 tests/gimbal_config_order_regression.py \
  --header Gimbal.hpp --algorithm YawLqrEso.hpp --header-only
bash tests/yaw_lqr_eso_host_regression.sh
clang-format --dry-run --Werror Gimbal.hpp
git diff --check
cd /home/sb/PLDX_Template
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/sentry_gimbal
```

Expected: all regressions exit 0 and firmware build ends with `Done.`.

- [ ] **Step 4: Review the semantic diff**

```bash
git -C Modules/Gimbal diff --word-diff=plain -- Gimbal.hpp
```

Confirm the diff changes only local fact capture and branch organization. It must not alter constants, topic names, signs, patrol formula, PID calls, motor commands, or AI references.

- [ ] **Step 5: Commit the ParseCMD refactor**

```bash
git add Gimbal.hpp tests/parse_cmd_behavior_regression.py
git commit -m "refactor(gimbal): deduplicate ParseCMD target updates"
```

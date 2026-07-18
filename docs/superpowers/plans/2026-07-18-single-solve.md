# Gimbal Single Solve Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Merge the two private Yaw solve helpers into the existing `Solve()` without changing control behavior.

**Architecture:** Keep `YawLqrEso` and motor submission boundaries intact. `Solve()` owns one Pitch block followed by an inline AI/Legacy Yaw branch.

**Tech Stack:** C++17, LibXR PID, static Bash regressions, STM32 cross-build.

## Global Constraints

- Preserve all current numerical formulas and update order.
- Preserve AI activation, LQR/ESO reset, invalid-output fallback, and applied-torque commit behavior.
- Do not change public APIs, constructor arguments, manifest, YAML, Topics, or motor commands.

---

### Task 1: Enforce one Solve function

**Files:**
- Modify: `Modules/Gimbal/tests/ai_yaw_integration_regression.sh`
- Modify: `Modules/Gimbal/Gimbal.hpp`

**Interfaces:**
- Consumes: existing `ai_yaw_active_`, PID state, `YawLqrEso`, sensor feedback, and target state.
- Produces: unchanged `pit_output_` and `yaw_output_` from one `Solve()` call.

- [ ] Add static checks requiring one `void Solve()` definition and forbidding `SolveAiYaw` and `SolveLegacyYaw`.
- [ ] Run the AI integration regression and confirm it fails on the old helpers.
- [ ] Inline both helper bodies into the existing Yaw branch in `Solve()` and remove the helper definitions.
- [ ] Run Gimbal core, AI integration, ParseCMD, controller host tests, format check, and sentry-gimbal build.

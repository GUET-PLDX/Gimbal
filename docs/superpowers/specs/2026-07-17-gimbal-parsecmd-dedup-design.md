# Gimbal ParseCMD Deduplication Design

## Scope

This work has two bounded parts:

1. Remove four state members that only preserve values between adjacent calls
   in the same control-loop iteration.
2. Plan a behavior-preserving simplification of `Gimbal::ParseCMD()` using
   local, phased calculations.

The snapshot-member removal is implemented first. The `ParseCMD()` rewrite is
planned separately and is not part of the first code change.

## Required Behavior

The refactor must preserve all current command semantics:

- Operator control integrates Pitch and Yaw commands as rates.
- Low-sensitivity operator control applies a `0.1f` multiplier to both axes.
- Active AI control sends the absolute Pitch command to the Pitch loop and
  leaves Yaw target generation to `YawLqrEso`.
- Automatic patrol preserves the existing Pitch patrol expression and the
  `+1.0f` Yaw rate.
- Non-AI automatic control preserves the current negative Yaw input direction.
- Pitch target processing remains before Yaw target processing.
- The retained motor-feedback guard, rotor feedforward, target derivative
  feedforward, and AI controller behavior remain unchanged.

## Snapshot Member Removal

Remove these members:

- `ctrl_mode_snapshot_`
- `ai_gimbal_status_snapshot_`
- `yaw_lqr_eso_config_snapshot_`
- `yaw_lqr_eso_output_`

Replace them as follows:

- Read control mode and AI status into uppercase local constants in
  `ParseCMD()`.
- Derive `AI_YAW_ACTIVE` from those local constants and retain only
  `ai_yaw_active_`, which is consumed later by `Solve()`.
- Use immutable `yaw_lqr_eso_config_` directly in `SolveAiYaw()`.
- Store the result of `YawLqrEso::Calculate()` in an uppercase local constant
  in `SolveAiYaw()`.

No public API, manifest field, YAML field, topic, or motor command changes.

## ParseCMD Structure

Use one function with three explicit phases:

1. Capture control facts as local constants:
   `CTRL_MODE`, `AI_GIMBAL_ACTIVE`, `AI_YAW_ACTIVE`, and mode predicates.
2. Update the Pitch target:
   - AI active: assign absolute position, velocity, and acceleration.
   - Automatic patrol: apply the existing patrol expression.
   - Otherwise: calculate one Pitch operator rate and integrate it.
3. Update the legacy Yaw target only when AI Yaw is inactive:
   - Operator control: integrate the input rate with the existing sensitivity.
   - Automatic patrol: integrate `+1.0f`.
   - Other automatic control: integrate the negated input rate.

This removes duplicated rate assignment without introducing helpers, policy
tables, or additional persistent state.

## Testing

Snapshot removal follows a red-green cycle:

1. Add a static regression that forbids the four member declarations and
   requires local control facts, direct configuration use, and a local AI
   output.
2. Run it against the current header and confirm the expected failure.
3. Apply the minimal member-to-local conversion.
4. Run the new regression, existing Gimbal static regressions, AI integration
   regression, LQR/ESO host tests, formatting checks, and the complete
   `sentry_gimbal` firmware build.

The later `ParseCMD()` rewrite requires characterization checks for every
branch listed under Required Behavior before production code changes.

## Risks

- Combining branches can accidentally change the sign of non-AI automatic Yaw
  input.
- A shared sensitivity calculation can incorrectly apply low sensitivity
  outside operator control.
- Reordering Pitch and Yaw work can change which command state reaches the
  controller in a loop iteration.

Tests must lock these details before the rewrite. No unrelated safety feature
or controller behavior is restored or removed as part of this work.

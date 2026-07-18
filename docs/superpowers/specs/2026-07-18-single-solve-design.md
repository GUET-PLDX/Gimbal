# Gimbal Single Solve Design

## Goal

Restore one private `Solve()` control-cycle function based on the QDU module
shape while preserving the current Pitch PID, Legacy Yaw PID, and AI Yaw
LQR/ESO behavior.

## Design

`Solve()` calculates Pitch first, then selects exactly one Yaw algorithm from
`ai_yaw_active_`. The Legacy Yaw and AI Yaw blocks remain behavior-identical
to the current `SolveLegacyYaw()` and `SolveAiYaw()` implementations, including
feedforward, reset lifecycle, invalid-output handling, and output assignment.

`YawLqrEso` remains an independent algorithm class. `ControlYawMotor()` remains
the sole Yaw actuator submission point and continues to commit only applied
torque. No constructor, manifest, YAML, Topic, mode, or motor behavior changes.

## Verification

Static integration coverage must require one `Solve()` definition, forbid the
two former helper definitions and calls, and preserve the AI/Legacy branch and
all existing torque submission assertions. Existing controller host tests and
the sentry-gimbal firmware build remain green.

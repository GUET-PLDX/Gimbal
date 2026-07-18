# Yaw Inertia Configuration Deduplication

## Goal

Use the original Gimbal `j_yaw` constructor parameter as the single Yaw
inertia source for both the legacy PID path and the AI Yaw LQR/ESO path.
Remove the duplicate externally configured `yaw_lqr_eso.j_kg_m2` value without
changing control routing or numerical behavior.

## Scope

- Remove `j_kg_m2` from `YawLqrEso::Config`.
- Remove `j_kg_m2` from the Gimbal manifest's `yaw_lqr_eso` mapping.
- Remove `j_kg_m2` from the sentry Gimbal robot configuration.
- Pass `j_yaw_` explicitly from `Gimbal::SolveAiYaw()` into
  `YawLqrEso::Calculate()`.
- Use the explicit inertia argument for acceleration feedforward and the ESO
  plant model.
- Update host tests, simulations, and configuration contract regressions for
  the new interface.

## Out Of Scope

- Unifying `yaw_k` with `b_nms_rad`.
- Unifying PID gains with LQR gains.
- Unifying legacy PID output limits with AI Yaw torque limits.
- Changing AI Yaw activation, fallback, reset, or torque submission behavior.
- Changing the legacy Pitch or Yaw control formulas.

## Interface Design

`YawLqrEso::Config` contains only controller tuning, observer tuning, optional
compensation, and output constraint fields. Plant inertia is supplied as a
required argument to validation and calculation:

```cpp
static bool ValidateConfig(const Config& config, float j_kg_m2);

Output Calculate(const Config& config, const Reference& reference,
                 const Feedback& feedback, float dt_s, float j_kg_m2);
```

`Gimbal::SolveAiYaw()` passes its existing `j_yaw_` member as `j_kg_m2`. The
algorithm rejects non-finite or non-positive inertia exactly as it currently
rejects an invalid `Config::j_kg_m2`.

## Data Flow

The robot YAML supplies `j_yaw` once. XRobot passes it to the Gimbal
constructor, which stores it in `j_yaw_`. The legacy path continues to use
`j_yaw_ * YAW_ALPHA`. The AI path passes the same `j_yaw_` to `Calculate()`,
which uses it for `tau_ff_alpha_nm`, `B0`, and the ESO viscous model term.

## Compatibility

The configured value is currently `0.03` for both `j_yaw` and
`yaw_lqr_eso.j_kg_m2`, so the refactor preserves the configured numerical
behavior. The `YawLqrEso` C++ API changes intentionally; all in-repository
callers and tests must migrate in the same commit.

## Verification

1. Add regression assertions that reject `j_kg_m2` in the external AI config
   and require `j_yaw_` at the AI calculation call.
2. Observe those assertions fail before production changes.
3. Update the algorithm, manifest, target YAML, and all host callers.
4. Run configuration, static integration, ParseCMD, host algorithm, physics,
   and simulation regressions.
5. Run the sentry Gimbal firmware build and confirm generated XRobot code uses
   the shortened aggregate in the declared field order.

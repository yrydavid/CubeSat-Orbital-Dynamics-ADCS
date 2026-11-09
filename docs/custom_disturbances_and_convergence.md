# Custom disturbances and numerical convergence

## Running

1. Open MATLAB R2025b and set Current Folder to repository root.
2. Execute `run_tests`.
3. Execute `run_demo` for the unchanged A/B magnetic unloading experiment.
4. Execute `run_disturbance_demo` for two optional example events.
5. Execute `run_convergence` to compare three integration and IGRF sampling grids.

## Disturbance events

Edit only `config/disturbance_events.m`:

- `type='torque'`: vector [N m] applied over `[time_s, time_s+duration_s)`.
- `type='impulse'`: angular impulse vector [N m s] at a precisely defined time.
- `frame='BODY'`: attached to spacecraft body, rotates with it.
- `frame='ECI'`: fixed inertial axes.
- Both event start and end must be aligned with `cfg.dt_s`, otherwise the simulator aborts with an actionable message.
- An event changes only attitude dynamics, not orbital translation. Impact fragments and translational velocity impulses are not modeled.
- The script uses a single held event selection per RK4 step to integrate a discontinuous finite torque without half-on boundary errors.
- For an ideal impulse, it applies `delta_omega_b=J^{-1}delta_H_b`, leaves quaternion and wheel momentum continuous, and adds `delta_H_eci` to the integrated external torque state.

All example amplitudes, directions and times are **original stress-test assumptions**; not flight telemetry. Do NOT describe them as measured impacts.

## Natural external torques

At t=0 the two environment disturbances are **computed**, not imposed constants:

`tau_gg = (3 mu/r^3) (rhat_body x J*rhat_body)`

`tau_res = m_res_body x B_body`.

- Gravity-gradient model: Celani (2021), Eq. 7, with instantaneous orbital radius replacing constant mean-radius simplification.
- Residual dipole assumption: Celani (2021), Section 4, `m_res=[0;0;3e-4] A m^2` adopted for a Tigrisat-inspired *simulation scenario*, not measured flight dipole.
- IGRF-14 field: IAGA/MathWorks `igrfmagm`; depends on epoch and orbital position.
- The attitude initial conditions and h wheel bias are original test assumptions.

Ref: https://doi.org/10.1007/s42496-020-00062-2

## Verification vs validation

- `run_tests` covers 3-D quaternion conventions, actuator limits, impulse jump and finite-pulse selection, and runs a short integrated smoke test.
- `run_convergence` compares `dt/envDt = 2/10, 1/5, 0.5/2.5 s` over the same physical interval. Compare switch times, final errors, wheel h, balance residuals. Passing asserts only establishes per-run numerical invariants, not converged results.
- Time-dependent switching and capacity clipping can create small non-smooth differences across grids. Do not claim convergence unless trends confirm it.
- Existing code uses a single detumble-to-pointing switch. It does NOT automatically revert to detumbling after a large event. Don't claim safe-mode re-entry or universal recovery.
- Rates and orientation are ideal feedback values; no sensor noise, actuator lag or current/voltage dynamics.

## Recommended GitHub layout

Keep `run_convergence.m`, `run_tests.m` and `tests` functions as **verification evidence**. Keep `run_disturbance_demo.m` and `config/disturbance_events.m` as final functionality. Do not upload historic `stepNN_*.m` experiments. Capture actual MATLAB numerical outputs in `docs/validation.md` only after verified on the final environment.

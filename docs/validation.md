# MATLAB R2025b verification and validation ledger

The figures and numerical outputs in this document were recorded from MATLAB R2025b runs on **8–9 November 2026**. They describe the reference configuration used for the project's first public release (software **1.0.0**). The physical equations and integration scheme are unchanged by the present documentation and event-metrics corrections. The added event-overlap regression tests have not been independently executed in MATLAB here; run `run_all` to check them locally. These are numerical checks of specified scenarios, not flight qualification or independent experimental validation.

## 1. Integrated tests

The pre-update `run_tests` execution **passed** the quaternion geometry, actuator-limit, end-to-end, event-selection, ECI impulse and `t=0` impulse bookkeeping tests in MATLAB R2025b. The current source adds additional cases for overlapping events; their outcome must be checked in MATLAB before reporting a complete test-suite pass.

## 2. Magnetic detumbling, inertial pointing, and wheel unloading

`run_demo` uses one common J2+IGRF environment and identical initial conditions for unloading ON/OFF. Test bias of wheel relative momenta `[2e-4;-1.5e-4;1e-4] N*m*s` is assumed, not flight telemetry.

| Output | Dumping ON | Dumping OFF |
|---|---:|---:|
| Automatic low-rate mode switch | 31.200 min | 31.200 min |
| Angular speed at switch | 0.026887 deg/s | 0.026887 deg/s |
| Final stored wheel-momentum norm | 2.28346826e-5 N*m*s | 3.03341101e-4 N*m*s |
| Final inertial pointing error | 0.054954 deg | 0.054954 deg |
| Final angular speed | 0.0005273 deg/s | 0.0005273 deg/s |
| Relative H-balance error, max | 1.669e-7 | 1.639e-7 |
| Peak nominal wheel-capacity usage | 100.00% | 100.00% |
| Number of output samples at wheel capacity | 17 (34 s sampled) | 18 (36 s sampled) |
| Last-orbit pointing error RMS | 0.17562486 deg | 0.17562486 deg |
| Motor-torque saturation at saved outputs | 0 | 0 |
| Dipole-command saturation at saved outputs | 0 | 0 |

ON reduces the final wheel-momentum norm by approximately 92.47% relative to OFF in this test. **A single final pointing error is not a bound on pointing accuracy over the orbit**. Peak wheel-capacity usage reaches its nominal operating cap temporarily, even though motor-torque saturation is not observed at saved output times.

## 3. Numerical convergence

`run_convergence` compares two grid scales simultaneously (attitude and environment). The **common final simulation time is 17210 s**, not the default 17216 s; therefore the convergence table's final pointing error is not numerically identical to `run_demo`.

| Attitude dt / environment dt | Final pointing error | Final wheel-momentum norm | Max relative H balance |
|---|---:|---:|---:|
| 2.0 s / 10.0 s | 0.05810015 deg | 2.292766e-5 N*m*s | 1.669e-7 |
| 1.0 s / 5.0 s | 0.05810015 deg | 2.292767e-5 N*m*s | 1.012e-8 |
| 0.5 s / 2.5 s | 0.05810015 deg | 2.292767e-5 N*m*s | 6.317e-10 |

The near-16x reduction of H residual for each 2x reduction of dt is consistent with fourth-order global integration behavior in this tested case. **These numbers combine numerical and environmental-grid refinement; they are not independent isolation of both errors**. The switch time remained 31.2 min on all grids. The number of sampled wheel-capacity points rises as the grid is refined, so it is not a comparable duration metric.

## 4. Custom external-disturbance recovery

`run_disturbance_demo` uses the default 3 nominal periods, an assumed finite **BODY torque** `[3e-6;-1e-6;0] N*m` at `5000..5020 s`, and an assumed ideal **ECI angular impulse** `[6e-5;0;0] N*m*s` at `8000 s`. These are original stress tests, **not measured spacecraft collisions**.

| Quantity | Torque pulse | Angular impulse |
|---|---:|---:|
| Peak full-3D attitude separation from clean baseline | 2.091276 deg | 1.993941 deg |
| Peak angular-rate difference | 0.065721 deg/s | 0.084079 deg/s |
| Peak wheel-momentum difference | 7.282155e-5 N*m*s | 8.633804e-5 N*m*s |
| First recovery below 0.1 deg **relative** for 600 s, counted after event end | 272 s | 278 s |

Angular-rate differences are computed after expressing both trajectories in ECI axes; the BODY frames generally differ after an event. The values in the table were recorded on 9 November 2026. The event-overlap classification and related tests were updated after those reference runs; rerun `run_tests` to verify that additional logic.

The recovery criterion is the angle between the event-driven attitude and the baseline's attitude **at the same time**. It does **not** claim absolute pointing error below 0.1 deg for a full orbit. Final pointing and last-orbit RMS matched the baseline to the displayed precision, despite nonzero transients.

## 5. Environmental model and plotted ranges

`run_environment_demo` produced three nominal orbital periods, approximately **4.782 h**. IGRF-14 magnitude ranged **17.971..48.966 microT**; geodetic altitude ranged **543.443..564.780 km**. 3D ECI orbit coloring and time histories were visually reviewed. The plotted geometry and magnetic-field ranges were examined for consistency, but no comparison with flight measurements was performed.

## 6. Initial natural external torques (computed, not manually imposed)

At t=0 in `run_disturbance_demo`:

- Gravity-gradient torque in BODY: `[+2.285074e-8; +3.720866e-8; +6.237921e-24] N*m`; norm `4.366510e-8 N*m`.
- Residual magnetic torque in BODY: `[+1.070204e-10; -2.193685e-9; 0] N*m`; norm `2.196294e-9 N*m`.

These are products of the instantaneous orbit, attitude, Celani's published test inertia/residual-dipole assumption, and the IGRF-14 field; there is no user-imposed constant initial natural torque.

## 7. Reproducing the reference case

From the repository root in MATLAB R2025b with Aerospace Toolbox, run `run_all` (or run the five `run_*.m` scripts individually). These commands exercise the reference configuration and regenerate the PNG files in `figures/`. Re-run and record the output for any changed conditions. The model does not include sensor errors, orbital velocity changes from impacts, nadir tracking or automatic return to detumbling after a large disturbance.

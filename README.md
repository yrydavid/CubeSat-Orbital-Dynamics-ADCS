# CubeSat Orbital Dynamics & Three-Axis Attitude Control

**MATLAB R2025b · Aerospace Toolbox · Release 1.0.0**

This project models a virtual 3U CubeSat in a J2-perturbed low-Earth orbit. It combines three-axis rigid-body attitude dynamics, quaternion-based inertial pointing, three reaction wheels and three magnetic torque rods. The geomagnetic field is computed along the orbit with IGRF-14. The main experiment compares reaction-wheel momentum accumulation with magnetic unloading enabled and disabled.

The model is an **academic numerical simulation**, not a flight-ready attitude determination and control system. Attitude and rate feedback are ideal; no sensor-based estimator is implemented.

## Run the complete project

Requirements: MATLAB R2025b and **Aerospace Toolbox** (including `igrfmagm`, `dcmeci2ecef`, `ecef2lla` and `dcmecef2ned`). No Simulink model is required.

1. Extract or clone the repository and open its **main folder** in MATLAB Current Folder.
2. Open `run_all.m` and press **Run**, or enter:

   ```matlab
   run_all
   ```

This runs the numerical tests first, then the environment, unloading comparison, convergence and disturbance simulations. All seven PNG figures are exported to `figures/`. If a test fails, execution stops with a MATLAB error instead of reporting an unverified result. The individual `run_tests.m` and `run_*_demo.m` / `run_convergence.m` scripts can also be executed separately.

## Simulation setup

- **Orbit:** initially circular, 550 km above the WGS84 equatorial radius, inclination 97.6°, propagated with central gravity and J2. Nominal duration: three Keplerian periods (about 4.782 h).
- **Magnetic environment:** IGRF-14 evaluated at interpolated spacecraft positions with time-dependent ECI/ECEF/WGS84/NED transformations.
- **Attitude state:** Hamilton scalar-first BODY-to-ECI quaternion, BODY angular velocity, three rotor angular momenta and an integrated external-torque ledger.
- **Controls:** rate-assisted magnetic detumbling, one-way automatic transition to fixed-ECI quaternion PD wheel pointing, plus optional magnetic momentum unloading.
- **Disturbances:** gravity-gradient and assumed residual magnetic torque, with optional finite body/inertial torques and instantaneous angular impulses.

The primary coupled dynamics are

$$
\mathbf H_b=\mathbf J\boldsymbol\omega+\mathbf h_w,\qquad
\dot{\mathbf h}_w=\mathbf u,
$$

$$
\mathbf J\dot{\boldsymbol\omega}=\boldsymbol\tau_{\mathrm{ext}}
-\mathbf u-\boldsymbol\omega\times(\mathbf J\boldsymbol\omega+\mathbf h_w),
\qquad \boldsymbol\tau_{\mathrm{mag}}=\mathbf m\times\mathbf B.
$$

Here `u` is motor torque applied to the **rotors**, with the opposite reaction on the spacecraft. Magnetic control torques cannot act parallel to the local magnetic field. [Mathematical model and assumptions](docs/model.md) gives the coordinate conventions, actuator limits, environment transforms and important simplifications.

## Reference-case results

The following values were recorded in MATLAB R2025b runs on **8–9 November 2026** for the reference configuration. They are simulated values, not hardware measurements. The updated event-overlap regression cases should be run locally through `run_all` before treating the revised test suite as passed.

| Quantity | Result |
|---|---:|
| Detumbling-to-pointing transition | 31.200 min |
| Final inertial pointing error | 0.054954° |
| Last-orbit RMS pointing error | 0.175625° |
| Final body angular-rate norm | 0.0005273°/s |
| Final rotor momentum norm, unloading ON | 2.28346826 × 10⁻⁵ N·m·s |
| Final rotor momentum norm, unloading OFF | 3.03341101 × 10⁻⁴ N·m·s |
| Reduction in final stored wheel momentum | **92.47%** |
| Maximum relative inertial-momentum balance residual (2 s grid) | 1.669 × 10⁻⁷ |

Both unloading cases briefly reach the chosen **nominal** momentum limit on an individual wheel. The close ON/OFF pointing match follows from the idealized feedforward compensation and should not be interpreted as a prediction of real flight performance. The momentum-balance residual is an **internal numerical consistency check**, not independent experimental validation.

![J2 orbit colored by IGRF-14 field magnitude](figures/orbital_environment.png)

![Geomagnetic field and geodetic coordinates](figures/geomagnetic_time_history.png)

![Magnetic momentum unloading comparison](figures/momentum_unloading_comparison.png)

![Actuator diagnostics](figures/actuator_diagnostics.png)

### Convergence and external events

`run_convergence` compares attitude/environment grids of **2/10**, **1/5** and **0.5/2.5 seconds**. The reported momentum-balance residuals were **1.669 × 10⁻⁷**, **1.012 × 10⁻⁸** and **6.317 × 10⁻¹⁰**, respectively. The trend is consistent with fourth-order behavior for the tested case, although the two grids are refined together and no universal order claim is made.

`run_disturbance_demo` applies two illustrative inputs, not measured collisions: a finite BODY torque at 5000–5020 s and an ECI angular impulse at 8000 s. The maximum 3D attitude separations from the disturbance-free baseline were **2.091276°** and **1.993941°**. Recovery to a **relative** attitude separation below 0.1° sustained for 600 s was reached after **272 s** and **278 s**, respectively, measured from the end of each event. This does not mean absolute pointing accuracy stayed below 0.1°.

![Grid convergence](figures/grid_convergence.png)

![Disturbance response](figures/disturbance_response.png)

![Relative attitude recovery](figures/disturbance_recovery.png)

[Full numerical results and test scope](docs/validation.md).

## Files and configuration

```text
CubeSat-Orbital-Dynamics-ADCS/
├── run_all.m               # one-command tests, demonstrations and figures
├── run_tests.m             # numerical and regression tests
├── run_environment_demo.m  # orbit/geomagnetic environment
├── run_demo.m              # unloading ON/OFF comparison
├── run_convergence.m       # grid sensitivity
├── run_disturbance_demo.m  # imposed torque/impulse study
├── config/                 # reference parameters, scenario, disturbance events
├── src/                    # orbit, environment, attitude, control, integration
├── docs/                   # mathematical model, sources, validation
└── figures/                # generated after running run_all
```

Ordinary scenario changes belong in `config/simulation_config.m`, hardware and controller parameters in `config/reference_cubesat.m`, and example disturbance events in `config/disturbance_events.m`. Re-run the tests and figures after changes. `run_demo` and related benchmarks intentionally override selected configuration values (for example, the initial wheel bias) so that the comparisons use controlled initial conditions.

## Sources and limitations

The spacecraft's reference inertia is adopted from **Celani (2021)**, while wheel and magnetic-torquer limits are drawn from **Astrofein RW1 Type A** and **CubeSpace CR0004** specifications. Geomagnetic modeling follows **IGRF-14** through the MathWorks Aerospace Toolbox. These are components of a **virtual** model, not specifications for one assembled spacecraft. Initial orbit, control gains and disturbance amplitudes are project choices. Educational background includes the 2026 bachelor's thesis *Attitude Control of a CubeSat* by Bianca-Andreea Topliceanu, whose separate laboratory inertia values are **not** used as this model's 3U inertia.

Source locators, parameter values and classifications are collected in [sources.md](docs/sources.md). Model assumptions and limitations, including ideal feedback, simplified Earth orientation, instantaneous ideal actuators, no drag/SRP and no safe-mode re-entry, are documented in [model.md](docs/model.md).

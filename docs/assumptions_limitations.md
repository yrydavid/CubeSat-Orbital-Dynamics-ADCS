# Assumptions, limitations and remaining work

## Numerical and mechanical conventions

1. BODY axes are aligned with three ideal orthogonal wheel/torquer axes. All body torques, dipoles, rates, and wheel momenta are expressed in BODY. `q` is scalar-first, active BODY -> ECI, Hamilton quaternion. The use of `quat2dcm` (MathWorks passive convention) would require a transpose.
2. `J` is interpreted as the **locked-assembly inertia** of a virtual 3U reference design. Rotor axial **relative** angular momentum is added separately. Since the published Tigrisat inertia and Astrofein wheel data come from different systems, assembly-level consistency is an assumption, not a known CAD configuration.
3. Nominal RW1 momentum capacity at 8000 rpm is used as a conservative **operating cap**. Published manufacturer maximum speed is higher (~16380 rpm), and nominal storage must not be described as the ultimate physical limit.
4. RW and MTQ commands change instantaneously within torque/dipole limits. Motor/coil electrical dynamics, bearing friction, mass imbalance, power, thermal, quantization, PWM and magnetic interference are not modeled.
5. Feedback uses *perfect* truth attitude, angular rate and model magnetic field, not estimated sensor readings. This is not flight-ready ADCS software.

## Orbit and geomagnetics

1. Environment is prescribed from a 550 km initial circular test orbit with 97.6-deg inclination; central gravity + rounded J2, no drag or solar radiation pressure.
2. IGRF-14 is evaluated from the geodetic WGS84 position and decimal UTC year. `decyear` is called with six **numeric** arguments to avoid the known zoned/unzoned datetime issue in the user's R2025b setup.
3. ECEF is converted using IAU-2000/2006 MathWorks `dcmeci2ecef`. We assume `UT1-UTC=0` and zero polar motion; J2 is expressed with simplified ECI Z symmetry. This is not a mission-grade Earth-orientation solution.
4. Between environment samples, ECI vectors are interpolated with shape-preserving cubic PCHIP, including the Runge-Kutta midpoint stages. A 2/10, 1/5, 0.5/2.5 s time-step/environment-step convergence study was completed by the project author in MATLAB; repeat it after material changes to gains, events, or model.

## Coupled attitude modes

1. `detumble`: `m = sat(kB*(omega x B_body))`, with RW motor torques disabled and momentum h held constant. This is an *ideal rate-assisted* rotational B-dot variant, not a measurement-only magnetometer implementation.
2. `point_dump`: an external MTQ command targets projected `-kDump*h`; wheel motors apply a magnetic-torque and gyroscopic-coupling feedforward plus quaternion-PD stabilization; both actuators saturate independently.
3. Automatic transition to inertial pointing occurs after 600 uninterrupted seconds with |omega| below 0.1 deg/s. The automatic transition was exercised in MATLAB and verified on the initial ON/OFF benchmark (31.2 min), but is not a complete mode supervisor. There is no high-rate safe-mode reentry. Scheduled mode is retained for regression tests only. Nadir tracking is not implemented.
4. The optional custom-disturbance module is implemented for finite torques and ideal attitude impulses. It does not alter orbital translation. If two events overlap, the coupled dynamics simulate their combined effect, but isolated per-event peak/recovery metrics are intentionally reported as undefined (NaN). Full 3D recovery metrics are reported in the event demo; no automatic high-rate safe-mode reentry exists.
5. Relative inertial-angular-momentum balance is checked by integrating the **external** torque in ECI in the same coupled state. This is a numerical self-consistency check, not an independent flight-data validation.

## Validated scenarios and remaining release checks

- MATLAB R2025b `run_tests` and `run_demo` return without errors on a clean MATLAB path.
- Inertial momentum balance, quaternion norm, motor/dipole/storage constraints, torque cross-product geometry.
- Results stable under `dt=2 s` -> `1 s` and environment grid `10 s` -> `5 s`.
- At least one demonstrably successful pointing-with-dumping case, plus controlled failure/saturation cases with honest interpretation.
- Automatic switch: verify in MATLAB and under alternative initial conditions; nadir tracking is not part of this scoped release.
- Custom pulse/impulse event scheduler, only if implemented and validated.
- All reproducibility figures and numerical metrics generated from the **final** integrated simulator, not imported from earlier step scripts.


## Automatic transition (v0.2 benchmark)

The mode-manager requires body angular speed below 0.1 deg/s continuously
for 600 seconds before switching from detumbling to inertial pointing.
This is an **assumed supervisory rule**, not a verified flight design.
It does not model re-entry to safe mode or attitude-estimation errors.

The unloading A/B test deliberately starts with nonzero wheel bias;
that value is selected for visibility of unloading and is not a flown
CubeSat data record.

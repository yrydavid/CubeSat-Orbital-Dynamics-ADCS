# Quick customization guide (for later detailed GitHub README)

The defaults are a **virtual 3U reference design**, not measured flight telemetry.

## Operating a demonstration

- `run_environment_demo` displays/export J2 orbit, IGRF-14 field, latitude, altitude.
- `run_demo` runs two *identical* initial states, with wheel unloading ON/OFF after detumbling.
- `run_disturbance_demo` reuses that integrated model and adds user events.
- `run_convergence` tests sensitivity to the numerical grid, and generates a figure.
- `run_tests` runs unit and short end-to-end physics tests.

Only top-level run_*.m files are executed. They create PNG plots in `figures/`.

## Change the basic scenario

Edit `config/simulation_config.m`:

- `cfg.numOrbits=3`: number of nominal Kepler periods.
- `cfg.h0_km=550`, `cfg.inclination_deg=97.6`: initial circular orbit; **project choices**.
- `cfg.utc0`: geomagnetic UTC epoch. Keep date within the model validity period and check leap seconds.
- `cfg.q0` and `cfg.omega0_rad_s`: initial orientation and BODY spin.
- `cfg.qTarget`: fixed inertial target unit quaternion. This model does not track nadir automatically.
- `cfg.hwheel0_Nms`: initial stored rotor momenta; each component must stay within nominal wheel capacity.
- `cfg.dt_s=2`, `cfg.envDt_s=10`: attitude and magnetic/environment sample grids.
- `cfg.detumbleRateThreshold_deg_s=0.1`, `cfg.detumbleDwell_s=600`: automatic transition criterion; supervisory assumptions.
- `cfg.enableDumping`: enabled/disabled in simulation. `run_demo.m` overrides this to run an explicit A/B pair.
- `cfg.events`: empty by default. Assigned from `disturbance_events()` in `run_disturbance_demo`.

The `cfg.finalTime_s` is computed from `numOrbits`, and each event time/duration must be a multiple of `dt_s`. User event timestamps must be inside the chosen simulation duration. For a shortened run, edit the event times too.

## User event input (config/disturbance_events.m)

A **finite moment** in BODY:

```matlab
events(1) = struct('type','torque','time_s',5000, ...
    'duration_s',20,'frame','BODY','vector',[3e-6;-1e-6;0]);
```

This torque vector uses **N*m**; nonzero for 20 seconds. `frame='ECI'` is also supported. Overlapping torque events sum.

An **ideal angular impulse** in ECI:

```matlab
events(2) = struct('type','impulse','time_s',8000, ...
    'duration_s',0,'frame','ECI','vector',[6e-5;0;0]);
```

The impulse vector uses **N*m*s** and changes body angular velocity instantaneously. It is not a torque expressed in deg/s. `frame='BODY'` is supported. To disable all user events, return `events=struct([]);`. No event values are claimed to be measured impact data.

## Physical reference parameters

`config/reference_cubesat.m` contains the reference inertia, RW1/CR0004 limits, and controller gains. Hardware limits are source-traced in `docs/parameter_sources.md`; change them only when intentionally testing a different documented actuator. Controller gains `Kp`, `Kd`, `kBdot`, `kDump` are original design choices and may be tuned, but retest stability and actuator limits after changes.

## Reporting results responsibly

Report **final** inertial pointing error and **last-orbit RMS** separately. Relative disturbance recovery is the angle between the no-event and event trajectories; it is not absolute pointing accuracy. Nominal wheel capacity and motor torque are separate constraints. `run_convergence` demonstrates numerical stability for tested initial conditions, not all imaginable inputs.

Current omissions: orbit feedback from impact, sensors, filtering, electrical coil response, air drag, SRP, thermal effects, nadir tracking, and safe-mode re-entry. Those are **not required** for the scoped academic simulator and should not be claimed.

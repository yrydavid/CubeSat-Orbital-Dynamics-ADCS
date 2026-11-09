# Mathematical model and assumptions

The implementation uses a prescribed J2 orbit, IGRF-14 field evaluation and a coupled three-axis rigid-body/reaction-wheel model. All attitude states and torques are evaluated in SI units. The orbital propagator uses km and km/s until the environmental interface.

## Orbital dynamics and magnetic field

For an ECI state `y = [r; v]`, the implemented acceleration is

```text
a = -(mu/r^3)*r + (3/2)*J2*mu*Re^2/r^5 * ...
    [x*(5*z^2/r^2-1); y*(5*z^2/r^2-1); z*(5*z^2/r^2-3)]
```

The initial circular reference orbit is 550 km above the WGS84 equatorial radius with 97.6-degree inclination; both are selected test conditions. The reference simulation spans three nominal Keplerian periods. The position is transformed ECI -> ECEF -> WGS84 geodetic coordinates. `igrfmagm(...,14)` evaluates IGRF-14 in local NED coordinates in nT; the resulting vector is converted NED -> ECEF -> ECI and multiplied by `1e-9` to obtain tesla. Environment samples are interpolated into the attitude integrator with PCHIP. Earth orientation uses the IAU-2000/2006 routines with UT1-UTC and polar motion set to zero as explicit approximations.

## Frames and quaternion convention

The quaternion `q = [q0; q1; q2; q3]` is scalar-first Hamilton, **BODY to ECI** (active rotation). `C = quat_to_dcm_b2i(q)` maps BODY-coordinate vectors into ECI; `C'` maps ECI to BODY. Angular velocity `omega` is expressed in BODY axes, with

```text
q_dot = 0.5 * q ⊗ [0; omega]
```

Quaternion normalization is enforced after each integration step. The reference pointing target is a fixed ECI orientation, not a nadir-following target.

## Rigid body and actuators

The adopted diagonal inertia `J = diag([0.0409, 0.0409, 0.0065]) kg m^2` comes from a published Tigrisat reference and is treated as the virtual assembly's locked inertia. `h` is the relative rotor angular momentum of three ideal orthogonal reaction wheels and `u` is motor torque *on the rotors*:

```text
H_body = J*omega + h
J*omega_dot = tau_external - u - cross(omega, J*omega + h)
h_dot = u
```

Magnetic torquer moment `m` produces `tau_MTQ = cross(m,B_body)`, perpendicular to `B_body`. Natural external torques are

```text
rhat_body = C'*(r_eci/norm(r_eci))
tau_GG = (3*mu_SI/r_m^3)*cross(rhat_body,J*rhat_body)
tau_residual = cross(m_residual,B_body)
tau_external = tau_GG + tau_residual + tau_MTQ + tau_custom
```

Per-axis wheel-motor torque, nominal wheel momentum and magnetic dipole limits are applied. The wheel's nominal 8000-rpm momentum specification is used as a conservative operational bound, not a maximum mechanical speed. A small numerical overshoot projection preserves `J*omega+h`.

## Modes and numerical integration

- **Detumbling:** `m = sat(kB*cross(omega,B_body))`, wheel motor torque zero. This is ideal rate-aided rotational B-dot, not an estimated magnetometer-only algorithm.
- **Pointing:** quaternion-error proportional/derivative control commands three wheel motor torques for a fixed ECI target.
- **Momentum unloading:** the magnetic torque requested from `-kDump*h` is projected onto the plane normal to `B_body`; wheel feedforward compensates the modeled magnetic torque. The ON/OFF experiment compares the two idealized controllers.
- **Switching:** automatic one-way detumble-to-pointing transition when `norm(omega) < 0.1 deg/s` for 600 consecutive seconds. These thresholds and gains are project choices, not flight-qualified parameters.

The attitude and wheel state uses fixed-step RK4 (2 s nominal) with interpolated magnetic/orbit environment samples (10 s nominal). The orbit is propagated with `ode45`. The inertial angular-momentum accounting check integrates the same external torques in ECI; it is an **internal consistency test**, not independent hardware validation.

## Prescribed disturbances

`config/disturbance_events.m` supports a finite torque in N m over `[time_s, time_s + duration_s)` or an ideal angular impulse in N m s at a grid-aligned instant. Both BODY and ECI vectors are accepted. Simultaneous finite torques add. An impulse changes body angular velocity by `J\Delta_H_body` but does not directly change quaternion or wheel momentum at the instant. Orbital translation is unchanged by these ideal attitude-only events. If events overlap, individual recovery metrics are not attributed separately; the combined trajectory remains valid.

## Limitations

This is a numerical **virtual reference configuration**, not an as-built spacecraft or flight-qualified ADCS. The body inertia (published Tigrisat reference) and actuator limits (independent vendor specifications) have not been validated together in CAD. Feedback uses perfect attitude, body rate and magnetic field; there is no sensor model, attitude estimator or magnetometer noise. The simulation excludes electrical coil dynamics, actuator lag and friction, wheel imbalance, magnetic self-interference, power/thermal behavior, drag, solar radiation pressure, attitude-dependent orbital forces, nadir tracking and automatic return to detumbling after a severe disturbance. Earth orientation is simplified. Numerical convergence for the reported scenario does not establish robustness for arbitrary configurations.

Parameter origins and source locators are listed in [sources.md](sources.md), and executed-case numbers are in [validation.md](validation.md).

# Mathematical model and conventions — final simulator

This document records the **implemented** mathematical equations, conventions, units, and interfaces. The 3D integration/control code is original to this virtual test design; it is not an implementation copied from Celani or from Topliceanu's thesis.

## Orbital environment

The propagated ECI Cartesian state is `y=[r_eci; v_eci]`, with position in km and velocity in km/s. Initial conditions: 550-km circular reference orbit, `i=97.6 deg`, `r0=[Re+550;0;0]`, and velocity derived from `sqrt(mu/r0)`. The acceleration is

`a = -(mu/r^3)*r + (3/2)*J2*mu*Re^2/r^5 * [x*(5*z^2/r^2-1); y*(5*z^2/r^2-1); z*(5*z^2/r^2-3)]`.

The nominal period is `2*pi*sqrt(r0^3/mu)`; three nominal periods are simulated by default. J2 makes successive revolutions not geometrically identical. Source: NASA Earth Fact Sheet for rounded `J2`; ESA GODOT/WGS84 for `Re`, `mu`. The orbit altitude/inclination are **project-selected**, not actual Tigrisat flight parameters.

At each environment sample, `dcmeci2ecef('IAU-2000/2006',UTC,...)` maps ECI to Earth-fixed coordinates. `ecef2lla(...,'WGS84')` gives geodetic latitude, longitude and altitude. `igrfmagm(height_m,lat_deg,lon_deg,decimal_year,14)` gives magnetic field in local NED in **nT**. `dcmecef2ned(...)'` maps NED->ECEF, and the ECI/ECEF DCM transpose maps ECEF->ECI. We explicitly convert `nT*1e-9` to tesla and km to meters before the attitude dynamics. The IGRF-14 model varies with position and date, not a static uniform vector. Environment interpolation at RK4 stages uses `pchip`. UT1-UTC and polar motion are assumed zero, not measured Earth-orientation parameters.

## Attitude and frames

`q = [q0;q1;q2;q3]` scalar-first Hamilton unit quaternion, **active BODY->ECI**. `C=quat_to_dcm_b2i(q)` maps body vector coordinates into ECI. `C'` maps ECI->BODY. Kinematics: `q_dot = (1/2) * q ⊗ [0;omega_body]`. Quaternions are normalized after each numerical time step. Attitude error uses the shortest-path equivalent quaternion (`q` and `-q` describe the same physical orientation) to command fixed inertial pointing to `q_target=[1;0;0;0]` by default; this is **not nadir-pointing**.

The published reference inertia `J=diag([0.0409;0.0409;0.0065]) kg m^2` comes from Celani, Eq. (31), for Tigrisat. Our virtual assembly uses it as an **assumed locked-assembly inertia**. Three orthogonal Astrofein RW1 Type A relative rotor momenta `h=[hx;hy;hz]` are modeled about BODY X/Y/Z without adding the rotors' locked inertias to `J` twice. No as-built mass/inertia budget is claimed.

## Coupled rigid body and three wheels

`H_body=J*omega_body+h_wheel`;
`J*omega_dot=tau_external - u - cross(omega_body,J*omega_body+h_wheel)`;
`h_dot=u`.

`u` is the motor torque applied to the **rotor** (the spacecraft body feels `-u`), bounded separately by `23e-6 N m` per Astrofein RW1 Type A. `|h_i|<=5.8e-4 N m s` is the chosen nominal-capacity operating limit (not the manufacturer's maximum mechanical rpm). Commands at the capacity limit that would charge farther are blocked, and a numerical projection at the end of a step preserves `J*omega+h` across any tiny overshoot. Command and capacity constraints are separate.

## Environmental torques and optional events

In BODY:

`rhat_body = C'*(r_eci/norm(r_eci))`;
`tau_GG = (3*mu_SI/r_m^3) * cross(rhat_body,J*rhat_body)`;
`B_body = C'*B_eci`;
`tau_residual = cross(m_residual_body,B_body)`;
`tau_MTQ = cross(m_applied_body,B_body)`;
`tau_external = tau_GG + tau_residual + tau_MTQ + tau_custom`.

The body-fixed residual dipole is `[0;0;3e-4] A m^2`, an example **simulation assumption** from Celani, not measured telemetry. In any fixed BODY frame, torque from a BODY-specified event is added directly; an ECI event torque is mapped into BODY by `C'` at each stage.

For a user-defined finite torque, `tau_custom` is held constant in its specified frame over each integration interval of its active duration, and overlapping pulses add. For a user-defined instantaneous angular impulse `Delta_H` at grid time `t0`, `omega(t0+)=omega(t0-)+inv(J)*Delta_H_body (conceptually; the MATLAB implementation uses J\Delta_H_body)`, while `q` and wheel `h` are continuous. An ECI impulse is rotated into BODY using `C'` at the instant; its inertial contribution is added to the momentum ledger. **Ideal impulses affect attitude dynamics, not orbital translation.**

## Control modes

- **DETUMBLE:** `m_req=k_B*(omega_body × B_body)`; `u=0`. This is an ideal body-rate-assisted rotational B-dot law (not finite-difference magnetometer-only B-dot). Torque `m×B` satisfies `omega·tau_MTQ<=0` in the ideal model even under symmetric independent per-axis saturation. It cannot instantaneously generate a torque parallel to `B`.
- **POINTING+UNLOADING:** `tau_dump_req = -k_dump*h`, then `m_req = cross(B_body,tau_dump_req)/(B_body·B_body)`; the actual torque is the attainable projected torque after per-axis dipole saturation. Independently, quaternion PD yields `tau_PD=Kp.*phi-Kd.*omega`. Wheel command `u_req=tau_MTQ-tau_PD-cross(omega,J*omega+h)` compensates the magnetic body moment and gyro coupling; environmental moments are not perfectly cancelled in this expression.
- **POINTING WITHOUT UNLOADING:** identical wheel PD but `m_req=0`. Comparing ON/OFF cases isolates magnetic unloading following the identical detumbling phase.

When `modePolicy='automatic'`, the system enters pointing once `|omega|<0.1 deg/s` for **600 consecutive seconds**, and stays there. There is no safe-mode re-entry after a severe event. The threshold/dwell and gains are **original design assumptions**, not a flight-certified algorithm.

## Numerical verification

Fixed RK4 integrates the attitude+wheel+ECI external-torque integral state with `dt_s=2 s` nominally. Orbital propagation uses `ode45` sampled at `envDt_s=10 s`; fields/positions are interpolated into attitude stages. A primary physics check is:

`H_ECI(t)-H_ECI(0-) = integral( C_body_to_ECI*tau_external_body dt ) + sum(Delta_H_ECI)`.

The model integrates the external-torque integral along the coupled state and includes instantaneous impulses. This is a **numerical consistency check**, not an independent proof of mechanical fidelity. Convergence is demonstrated by comparing `dt/envDt = 2/10, 1/5, 0.5/2.5 s`, and the results must be cited as scenario-specific.

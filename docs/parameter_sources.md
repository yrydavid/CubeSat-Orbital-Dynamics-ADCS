# Parameter provenance and bibliography

All reference values are in SI units inside the coupled attitude model; orbital integrator state uses **km** and **km/s** and is converted at the environment boundary.

| Item in code | Quantity | Source / locator | Classification |
|---|---|---|---|
| `p.J` | `diag([0.0409,0.0409,0.0065]) kg m^2` | Fabio Celani (2021), *Gain Selection for Attitude Stabilization of Earth-Pointing Spacecraft Using Magnetorquers*, Eq. (31), https://doi.org/10.1007/s42496-020-00062-2 | **Published** inertia for Tigrisat reference; adopted for virtual design |
| `p.mResidual` | `[0;0;3e-4] A m^2` | Celani (2021), Sec. 4 (perturbed simulation) | **Published simulation assumption**, NOT flight measured |
| `p.Iwheel` | `694.5e-9 kg m^2` per wheel | Astrofein RW1 Type A technical product table, https://www.astrofein.com/en/reaction-wheels/reaction-wheel-rw1/ | Manufacturer |
| `p.hMax` | `5.8e-4 N m s` per wheel | Same Astrofein RW1 Type A table; nominal at 8000 rpm | Manufacturer; **nominal operating cap** |
| `p.uMax` | `23e-6 N m` per wheel | Same Astrofein RW1 Type A table | Manufacturer; max commandable nominal |
| `p.mMax` | `0.4 A m^2` per axis | CubeSpace *CubeTorquer Product Description*, document CS-DEV.PD.CR-01, v2.00, 24 Nov 2025, **Table 3 (p. 7)**, CR0004 column, https://www.cubespace.co.za/uploads/b6f0983d-13fa-4cc2-92d9-5b52ac37dcc8.pdf | Manufacturer; rated dipole, *not* an arbitrary total-vector norm cap |
| `igrfmagm(...,14)` | Geomagnetic field in NED, **nT** | MathWorks `igrfmagm` official documentation, https://www.mathworks.com/help/aerotbx/ug/igrfmagm.html ; IAGA IGRF-14 | Official model + software implementation |
| `dcmeci2ecef`, `ecef2lla`, `dcmecef2ned` | ECI-ECEF-WGS84-NED transforms | MathWorks Aerospace Toolbox reference pages | Official software APIs |
| `cfg.Re_km`, `cfg.mu_km` | 6378.137 km and 398600.4418 km^3/s^2 | ESA GODOT WGS84 Earth constants, https://godot.io.esa.int/docs/api_reference/generated/generated/godot.core.constants.html (original WGS84 standard issued by NGA) | Geodetic standard |
| `cfg.J2` | `1.08263e-3` | NASA Earth Fact Sheet J2 = 1082.63 x 10^-6: https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html | Published rounded constant; exact normalization/version must be frozen for release |
| `cfg.h0_km`, `cfg.inclination_deg` | 550 km, 97.6 deg | This project's selected test orbit; **not** Tigrisat's orbit | Assumption |
| `cfg.utc0` | 2026-11-06T00:00 UTC | Selected epoch; TAI-UTC `37 s` based on IERS Bulletin C 72 | Chosen epoch + published time standard |
| `p.kBdot`, `p.kDump`, `p.Kp`, `p.Kd` | Controller gains in `reference_cubesat.m` | Original illustrative design, to be tuned by simulation | **Design assumptions** |
| `cfg.omega0_rad_s`, `cfg.q0` | Initial rates, axis-angle | Original chosen test case | **Assumptions**, not flight telemetry |

## Formulas and provenance

- Rigid-body Euler equation, quaternion kinematics, gravity-gradient torque and magnetic torque are standard rigid-body/ADCS theory. Compare **Celani (2021), Eqs. (6), (7), (9)** and the original standard references cited in the paper.
- The reaction-wheel momentum extension `H_b = J*omega_b + h_wheels`, `h_dot = u` is standard conservation of total angular momentum; motor torque is **internal**, magnetic torque is **external**. The specific coupled implementation and feedback modes are original to this repository and must be tested separately.
- The source **Bianca-Andreea Topliceanu, *Attitude Control of a CubeSat* (2026)**, user-held thesis, Secs. 4.1.3, 4.2.2, 4.3.2, 8.5.1 and 12.3, motivates the transition from one-axis laboratory dynamics to three-axis attitude control. Its Fusion 360 moments `0.000634` and `4.607e-5 kg m^2` characterize a **different demonstrator**, and are deliberately **not** used as the 3U body or RW1 inertia.
- The public Politecnico di Milano *Spacecraft Attitude Dynamics* syllabus is relevant curriculum context, **not the claimed origin of source code or unretrieved lecture notes**.

Do not include third-party PDFs in the repository unless redistribution is permitted. Cite authoritative URLs and exact equation/table identifiers instead.

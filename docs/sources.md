# Sources and parameter provenance

The spacecraft is a **virtual research configuration**: the cited paper, vendor specifications and assumptions do **not** describe one assembled or flight-tested CubeSat. Literature values, manufacturer limits and project-selected tuning values are separated below.

| Model input | Value | Provenance |
|---|---|---|
| `p.J` | `diag([0.0409,0.0409,0.0065]) kg m^2` | Celani (2021), Eq. 31, published Tigrisat reference inertia; assumed locked inertia of the virtual configuration |
| `p.mResidual` | `[0;0;3e-4] A m^2` | Celani (2021), Sec. 4, **published simulation assumption**, not measured flight data |
| `p.Iwheel` | `694.5e-9 kg m^2` per wheel | Astrofein RW1 Type A manufacturer data |
| `p.hMax` | `5.8e-4 N m s` per wheel | Astrofein RW1 Type A nominal 8000-rpm momentum; chosen operating cap, not mechanical maximum |
| `p.uMax` | `23e-6 N m` per wheel | Astrofein RW1 Type A maximum commandable torque |
| `p.mMax` | `0.4 A m^2` per axis | CubeSpace CubeTorquer CR0004 product description, v2.00, Table 3 (p. 7); ideal three-unit orthogonal arrangement assumed |
| `cfg.Re_km`, `cfg.mu_km` | `6378.137 km`, `398600.4418 km^3/s^2` | WGS84 values, ESA GODOT/NGA |
| `cfg.J2` | `1.08263e-3` | NASA Earth Fact Sheet, rounded coefficient |
| `igrfmagm(...,14)` | IGRF-14 magnetic field in NED, nT | IAGA IGRF-14 model evaluated via MathWorks Aerospace Toolbox |
| `cfg.utc0` | 6 Nov 2026 00:00 UTC | Selected epoch; 37 s TAI-UTC from IERS Bulletin C 72 |
| Orbit, initial state, gains, event vectors, switching thresholds | See `config/*.m` | Original **simulation choices**, not hardware measurements or published mission telemetry |

## Primary references and official specifications

1. Celani, F. (2021). *Gain Selection for Attitude Stabilization of Earth-Pointing Spacecraft Using Magnetorquers*. *Aerotecnica Missili & Spazio*, **100**, 15–24. [DOI: 10.1007/s42496-020-00062-2](https://doi.org/10.1007/s42496-020-00062-2). Inertia Eq. 31; gravity-gradient Eq. 7; residual dipole assumption Sec. 4.
2. Beggan, C. D., et al. (2026). *International geomagnetic reference field: the fourteenth generation*. *Earth, Planets and Space*, **78**, 127. [DOI: 10.1186/s40623-025-02360-0](https://doi.org/10.1186/s40623-025-02360-0).
3. MathWorks. [Aerospace Toolbox `igrfmagm`](https://www.mathworks.com/help/aerotbx/ug/igrfmagm.html). Field in NED/nT; WGS84 and epoch transforms are evaluated with Aerospace Toolbox functions.
4. Astro- und Feinwerktechnik Adlershof GmbH. [RW1 Type A technical specifications](https://www.astrofein.com/en/reaction-wheels/reaction-wheel-rw1/).
5. CubeSpace. [*CubeTorquer Product Description*, v2.00 (24 Nov 2025), Table 3](https://www.cubespace.co.za/uploads/b6f0983d-13fa-4cc2-92d9-5b52ac37dcc8.pdf). CR0004 magnetic dipole specification.
6. ESA GODOT. [Earth constants](https://godot.io.esa.int/docs/api_reference/generated/generated/godot.core.constants.html).
7. NASA GSFC. [Earth Fact Sheet](https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html).
8. IERS. [Bulletin C 72](https://datacenter.iers.org/data/html/bulletinc-072.html), UTC-TAI offset for selected epoch.
9. NASA (2026). [*State-of-the-Art of Small Spacecraft Technology — Guidance, Navigation, and Control*](https://www.nasa.gov/smallsat-institute/sst-soa/guidance-navigation-and-control/). General ADCS context; not a validation of this simulator.

## Educational context

Bianca-Andreea Topliceanu, *Attitude Control of a CubeSat* (bachelor's thesis, 2026), is acknowledged for educational background in reference frames, single-axis control and reaction-wheel dynamics. The thesis's Fusion 360 inertias (`0.000634` and `4.607e-5 kg m^2`, Sec. 8.5.1) describe **another** laboratory demonstrator and are **not** used for the virtual 3U inertia or Astrofein wheel rotor. The [IIT Kharagpur/NPTEL course *Satellite Attitude Dynamics and Control*](https://nptel.ac.in/courses/101105077) is an additional educational reference. Neither source is redistributed as a file or presented as having supplied this repo's 3D code.

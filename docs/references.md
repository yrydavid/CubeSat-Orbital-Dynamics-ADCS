# Verified sources and provenance for ADCS integration

## Research and physical models

- **Celani, F. (2021).** *Gain Selection for Attitude Stabilization of
  Earth-Pointing Spacecraft Using Magnetorquers.* Aerotecnica Missili &
  Spazio 100, 15–24. DOI: https://doi.org/10.1007/s42496-020-00062-2.
  Used: Tigrisat 3U principal inertia `diag([0.0409 0.0409 0.0065])`
  kg m^2 (Eq. 31), rigid-body Euler equations (Eq. 6) and magnetic
  torque geometry (Eq. 9). This is NOT the flown Tigrisat model.
- **IAGA/IGRF-14** as implemented in **MathWorks MATLAB R2025b Aerospace
  Toolbox** `igrfmagm`: https://www.mathworks.com/help/aerotbx/ug/igrfmagm.html.
  Position/time dependent geomagnetic model.
- **ESA GODOT** Earth constants: https://godot.io.esa.int/docs/api_reference/generated/generated/godot.core.constants.html.
  Selected constants compatible with WGS84; orbit 550 km, i=97.6 deg
  and J2 truncation are project choices.

## Manufacturer specifications

- **Astrofein RW1 Type A**, https://www.astrofein.com/en/reaction-wheels/reaction-wheel-rw1/.
  Rotor inertia 694.5e-9 kg m^2, 23e-6 N m motor max-commandable,
  5.8e-4 N m s rated momentum storage (nominal, not ultimate capacity).
- **CubeSpace CubeTorquer CR0004** product description (Nov 2025):
  https://www.cubespace.co.za/uploads/b6f0983d-13fa-4cc2-92d9-5b52ac37dcc8.pdf.
  Nominal dipole magnitude 0.4 A m^2 at 5 V per actuator. This model
  assumes three orthogonal units with independent symmetric dipole commands.

## Engineering overview and educational acknowledgements

- **NASA Small Spacecraft Technology State-of-the-Art (2026)**,
  Chapter 5, Guidance Navigation and Control, Sec. 5.2.2–5.2.3:
  https://www.nasa.gov/smallsat-institute/sst-soa/guidance-navigation-and-control/.
  Supports architecture reasoning and external magnetic momentum unloading;
  NOT an original source of manufacturer-specific limits.
- **Bianca-Andreea Topliceanu (2026)**, *Attitude Control of a CubeSat*,
  bachelor thesis; Ch. 4 (reference frames/quaternions), Sec. 8.5.1
  (lab-demonstrator inertias), Sec. 12.3 (proposed future 3D/magnetic control).
  No thesis code or lab inertia is represented as our 3U inertia.
- **GIAN–MHRD, IIT Kharagpur**, video lecture *13. Attitude Control
  Using Magneto Torquers*, https://www.youtube.com/watch?v=WfDf9CHWS_E&t=3428s.
  Acknowledged as a learning resource, NOT parameter source.

## Clearly original/assumed

- Initial wheel momentum `[2e-4;-1.5e-4;1e-4] N m s`;
  low-rate threshold 0.1 deg/s sustained for 600 s; gain matrices and
  dumping gain; assumed ideal angular rate and attitude feedback.
- Locked-assembly inertia interpretation for mixed-source virtual 3U
  with added relative rotor momentum (must be distinguished from measured
  as-built spacecraft inertia).


### Disturbance-model provenance (explicit)

- Celani (2021), **Eq. (7)** is the gravity-gradient torque. Its amplitude in our code is recomputed from `r_ECI`, body attitude and inertia; it is **not** an imposed constant.
- Celani (2021), **Section 4** uses `T_dist = m_0 × b_body` and selects `m_0=[0 0 3×10^-4]^T A m²` as a **realistic simulation assumption**, not as a measured Tigrisat magnetic moment.
- Celani (2021), **Eq. (31)** gives principal moments of inertia; footnote 2 credits personal communication, which limits independent traceability of that value.
- Optional `disturbance_events.m` examples (20-s finite torque, angular impulse) are **original numerical test cases** and not extracted from measured impacts.

## Additional primary-source verification

- NASA/GSFC Earth Fact Sheet: https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html. Earth J2 (x 10^-6) = 1082.63; this supports the rounded J2 coefficient 1.08263e-3 employed in the propagator.
- CubeSpace CubeTorquer *Product Description*, document CS-DEV.PD.CR-01, v2.00, 24 November 2025, p. 7, Table 3. This is distinct from the separately published NanoSat *Interface Control Document* (ICD v1.03).
- Celani's case-study Eq. (31) and Section 4 provide source values; the physical wheel-PD and magnetic unloading algorithms in THIS repository are original modeling decisions, not implementations copied from Celani.

## Additional exact bibliographic and educational sources

- **Beggan, C. D., et al. (2026).** *International geomagnetic reference field: the fourteenth generation*. Earth, Planets and Space 78, 127. DOI: https://doi.org/10.1186/s40623-025-02360-0. This is the original scientific model behind the MathWorks IGRF-14 software call, not a description of our controller.
- **NASA / GSFC**, *Earth Fact Sheet*: https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html. Tabulates J2 as `1082.63 × 10^-6`.
- **ESA GODOT**, *WGS84 Earth Constants*: https://godot.io.esa.int/docs/api_reference/generated/generated/godot.core.constants.html. Directly lists equatorial semi-major axis and Earth GM used in the code.
- **NASA / GSFC CCMC**, *Dipole Field Equations*: https://ccmc.gsfc.nasa.gov/static/files/Dipole.pdf. Used only during the earlier dipole-based educational/verification work, **not** the final IGRF-14 physical field.

### Provenance status of the source author's thesis

Bianca-Andreea Topliceanu's student thesis is user-provided academic background. It is not redistributed in this project and is not claimed to be the source of the 3D MATLAB implementation, RW1 parameters, or the Tigrisat inertia tensor. The original thesis references were audited separately; only directly relevant sources are carried over to the final bibliography. The GIAN-MHRD IIT Kharagpur magnetorquer lecture is a clearly attributed educational acknowledgment, not an actuator calibration source.

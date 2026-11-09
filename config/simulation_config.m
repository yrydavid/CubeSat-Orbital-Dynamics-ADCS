function cfg = simulation_config()
%SIMULATION_CONFIG Project-selected conditions, distinct from published data.
cfg.mu_km = 398600.4418;             % km^3/s^2, WGS84
cfg.Re_km = 6378.137;                % km, WGS84 equatorial radius
cfg.J2 = 1.08263e-3;                 % rounded Earth J2
cfg.h0_km = 550;                     % selected initial circular altitude
cfg.inclination_deg = 97.6;         % selected orbital inclination
cfg.utc0 = datetime(2026,11,6,0,0,0,'TimeZone','UTC');

r0 = cfg.Re_km + cfg.h0_km;
v0 = sqrt(cfg.mu_km/r0);
cfg.y0_orbit_km = [r0;0;0;0;v0*cosd(cfg.inclination_deg); ...
    v0*sind(cfg.inclination_deg)];
cfg.orbitPeriod_s = 2*pi*sqrt(r0^3/cfg.mu_km);

% One common integration grid. Earth/magnetic environment every 10 s.
cfg.dt_s = 2;                         % attitude numerical step [s]
cfg.envDt_s = 10;                    % environment sampling step [s]
cfg.numOrbits = 3;                   % nominal Kepler periods in each demo
cfg.finalTime_s = round(cfg.numOrbits*cfg.orbitPeriod_s/cfg.dt_s)*cfg.dt_s;
cfg.detumbleEnd_s = round(1.0*cfg.orbitPeriod_s/cfg.dt_s)*cfg.dt_s;

% Body->ECI quaternion, chosen initial attitude and rates.
a = [0.3;0.5;0.8]; a = a/norm(a);
cfg.q0 = [cosd(20);a*sind(20)];      % 40-deg initial rotation
cfg.omega0_rad_s = deg2rad([0.7;-0.4;1.2]);
cfg.hwheel0_Nms = zeros(3,1);
cfg.qTarget = [1;0;0;0];            % inertial pointing target for MVP

% Flag for comparison; false = pointing without wheel unloading.
cfg.enableDumping = true;
% Optional custom events (disabled by default; see disturbance_events.m).
cfg.events = struct([]);

% Automatic switch from detumbling to inertial pointing.
% Transition only after the body rate stays below the threshold for
% a continuous dwell time. Never switch because of one isolated sample.
% [ASSUMED] Supervisory settings; not a published flight controller.
cfg.modePolicy = 'automatic';
cfg.detumbleRateThreshold_deg_s = 0.10;
cfg.detumbleDwell_s = 600;
cfg.minimumDetumble_s = 0;

% Earth orientation: TAI-UTC validated for this date. UT1/polar motion
% approximated zero; see docs/assumptions_limitations.md.
cfg.deltaAT_s = 37;
cfg.deltaUT1_s = 0;
cfg.polarMotion_rad = [0 0];

assert(isfinite(cfg.numOrbits) && isscalar(cfg.numOrbits) && ...
    cfg.numOrbits>0 && cfg.finalTime_s>0, ...
    'The requested number of orbits must produce a positive duration.');
if strcmp(cfg.modePolicy,'scheduled')
    assert(cfg.detumbleEnd_s < cfg.finalTime_s, ...
        'A scheduled mode switch must occur before the simulation ends.');
end
assert(mod(cfg.envDt_s,cfg.dt_s)==0);
end

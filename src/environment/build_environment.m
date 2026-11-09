function env = build_environment(cfg)
%BUILD_ENVIRONMENT Compute J2 orbit and IGRF-14 field in ECI.
% Requires MATLAB R2025b + Aerospace Toolbox.
% IGRF: geodetic height in meters, field NED in nT; convert to Tesla.

tEnv = unique([(0:cfg.envDt_s:cfg.finalTime_s)'; cfg.finalTime_s]);
opt=odeset('RelTol',1e-10,'AbsTol',1e-12);
[~,Y]=ode45(@(t,y)j2_rhs(t,y,cfg.mu_km,cfg.Re_km,cfg.J2), ...
    tEnv,cfg.y0_orbit_km,opt);
rI=Y(:,1:3)*1e3;
vI=Y(:,4:6)*1e3;
N=numel(tEnv);

utc=cfg.utc0+seconds(tEnv);
Cei=dcmeci2ecef('IAU-2000/2006',utc, ...
    cfg.deltaAT_s*ones(N,1),cfg.deltaUT1_s*ones(N,1), ...
    repmat(cfg.polarMotion_rad,N,1));
rE=zeros(N,3);
for k=1:N
    rE(k,:)=(Cei(:,:,k)*rI(k,:)')';
end
lla=ecef2lla(rE,'WGS84');
yearDecimal=decyear(year(utc),month(utc),day(utc), ...
    hour(utc),minute(utc),second(utc));
[Bned_nT,~,~,~,F_nT]=igrfmagm( ...
    lla(:,3),lla(:,1),lla(:,2),yearDecimal,14);

assert(max(abs(vecnorm(Bned_nT,2,2)-F_nT))<1e-5, ...
    'IGRF-14 NED vector and magnitude disagree.');
Beci=zeros(N,3);
for k=1:N
    C_e_n = dcmecef2ned(lla(k,1),lla(k,2))';
    Beci(k,:)=(Cei(:,:,k)'*(C_e_n*Bned_nT(k,:)'*1e-9))';
end
assert(all(isfinite(Beci(:))) && all(lla(:,3)>0));

env.t=tEnv;
env.r_eci_m=rI;
env.v_eci_m=vI;
env.B_eci_T=Beci;
env.altitude_m=lla(:,3);
env.latitude_deg=lla(:,1);
env.field_uT=vecnorm(Beci,2,2)*1e6;
end

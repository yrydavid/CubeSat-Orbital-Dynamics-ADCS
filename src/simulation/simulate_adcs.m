function out=simulate_adcs(cfg,p,env)
%SIMULATE_ADCS Fixed-step RK4 integration of coupled 3D attitude+RW model.
% Mode policy: AUTOMATIC low angular-rate dwell, or SCHEDULED for tests.
% Detumbling uses ideal angular-rate feedback; pointing uses ideal attitude
% feedback. Once POINTING starts, it remains active (no safe-mode re-entry).
% Wheel unloading ON/OFF is controlled by cfg.enableDumping.
% This is not an autonomous fault-management system or flight software.

dt=cfg.dt_s;
if ~isfield(cfg,'events') || isempty(cfg.events)
    cfg.events=struct([]);
end
validate_disturbance_events(cfg.events,cfg);
t=(0:dt:cfg.finalTime_s)';
N=numel(t);
r=interp1(env.t,env.r_eci_m,t,'pchip');
B=interp1(env.t,env.B_eci_T,t,'pchip');
tMid=t(1:end-1)+0.5*dt;
rMid=interp1(env.t,env.r_eci_m,tMid,'pchip');
BMid=interp1(env.t,env.B_eci_T,tMid,'pchip');
assert(all(isfinite(r(:))) && all(isfinite(B(:))) && ...
    all(isfinite(rMid(:))) && all(isfinite(BMid(:))));

x=zeros(N,13);
x(1,1:3)=cfg.omega0_rad_s';
x(1,4:7)=cfg.q0';
x(1,8:10)=cfg.hwheel0_Nms';
% Store the physical momentum BEFORE any user impulse at t=0.
% Otherwise the first saved state is already post-impulse, and adding
% the impulse integral to that state double-counts the event.
C_initial=quat_to_dcm_b2i(cfg.q0);
H0_eci_pre_events=C_initial*(p.J*cfg.omega0_rad_s+cfg.hwheel0_Nms);
% State components 11:13 integrate external torque in ECI.

% Auto mode: requires a continuous low-rate dwell before switching.
% Scheduled mode is retained solely as a regression test option.
mode=cell(N,1);
currentMode='detumble';
lowRateElapsed_s=0;
previousSampleBelow=false;
switchTime_s=NaN;
switchReason='not reached';


for k=1:N-1
    % Apply all ideal angular impulses at this exact sample time.
    % Add the same impulse in ECI to the angular-momentum integral.
    [xApplied,~]=apply_impulses_at_time(x(k,:)',cfg.events,t(k),p);
    x(k,:)=xApplied';

    if strcmp(cfg.modePolicy,'automatic')
        if strcmp(currentMode,'detumble')
            rateNow=rad2deg(norm(x(k,1:3)));
            if rateNow < cfg.detumbleRateThreshold_deg_s
                if previousSampleBelow
                    lowRateElapsed_s=lowRateElapsed_s+dt;
                else
                    lowRateElapsed_s=0;
                end
                previousSampleBelow=true;
            else
                lowRateElapsed_s=0;
                previousSampleBelow=false;
            end
            if t(k)>=cfg.minimumDetumble_s && ...
                    lowRateElapsed_s>=cfg.detumbleDwell_s
                currentMode='pointing';
                switchTime_s=t(k);
                switchReason='rate threshold held for dwell time';
            end
        end
    elseif strcmp(cfg.modePolicy,'scheduled')
        if t(k)>=cfg.detumbleEnd_s && strcmp(currentMode,'detumble')
            currentMode='pointing';
            switchTime_s=t(k);
            switchReason='scheduled time';
        end
    else
        error('Unknown modePolicy: %s',cfg.modePolicy);
    end

    if strcmp(currentMode,'detumble')
        modeStep='detumble';
    elseif cfg.enableDumping
        modeStep='point_dump';
    else
        modeStep='point_no_dump';
    end
    mode{k}=modeStep;
    rk=r(k,:)'; bk=B(k,:)';
    rn=r(k+1,:)'; bn=B(k+1,:)';
    rm=rMid(k,:)'; bm=BMid(k,:)';
    xk=x(k,:)';
    % Hold event selection constant on each RK4 interval.
    % This avoids evaluating discontinuities at an RK4 boundary.
    tauEvent=custom_torque_at_midpoint(cfg.events,tMid(k));

    k1=adcs_rhs(xk,rk,bk,modeStep,p,cfg,tauEvent);
    k2=adcs_rhs(xk+0.5*dt*k1,rm,bm,modeStep,p,cfg,tauEvent);
    k3=adcs_rhs(xk+0.5*dt*k2,rm,bm,modeStep,p,cfg,tauEvent);
    k4=adcs_rhs(xk+dt*k3,rn,bn,modeStep,p,cfg,tauEvent);
    xNew=xk+(dt/6)*(k1+2*k2+2*k3+k4);

    % Numerical projection only at a hard wheel-capacity boundary.
    % Preserve BODY angular momentum when clipping rotor h.
    hBefore=xNew(8:10);
    hAfter=max(-p.hMax,min(p.hMax,hBefore));
    xNew(1:3)=xNew(1:3)+p.J\(hBefore-hAfter);
    xNew(8:10)=hAfter;
    xNew(4:7)=xNew(4:7)/norm(xNew(4:7));
    x(k+1,:)=xNew';
end

mode{N}=mode{N-1};
omega=x(:,1:3);
q=x(:,4:7);
h=x(:,8:10);
Heci=zeros(N,3);
torqueExternal=zeros(N,3);
torqueMag=zeros(N,3);
torqueMotor=zeros(N,3);
magDipole=zeros(N,3);
rate=zeros(N,1);
errorDeg=zeros(N,1);
qNormError=zeros(N,1);
dipoleSaturated=false(N,1);
motorSaturated=false(N,1);

for k=1:N
    xk=x(k,:)';
    diagnosticTime=min(t(k)+0.5*dt,t(end)-0.5*dt);
    diagnosticTorque=custom_torque_at_midpoint(cfg.events,diagnosticTime);
    [~,d]=adcs_rhs(xk,r(k,:)',B(k,:)',mode{k},p,cfg,diagnosticTorque);
    C=quat_to_dcm_b2i(xk(4:7));
    Heci(k,:)=(C*(p.J*xk(1:3)+xk(8:10)))';
    torqueExternal(k,:)=d.tauExternal';
    torqueMag(k,:)=d.tauMag';
    torqueMotor(k,:)=d.u';
    magDipole(k,:)=d.m';
    rate(k)=rad2deg(norm(xk(1:3)));
    [~,err]=attitude_error(xk(4:7),cfg.qTarget);
    errorDeg(k)=rad2deg(err);
    qNormError(k)=abs(norm(xk(4:7))-1);
    dipoleSaturated(k)=d.dipoleSaturated;
    motorSaturated(k)=d.motorSaturated;
end

% Compare against the physical momentum before any user impulse,
% including a possible ideal impulse applied exactly at t=0.
H_expected=repmat(H0_eci_pre_events',N,1)+x(:,11:13);
H_err=vecnorm(Heci-H_expected,2,2);
H_scale=max(norm(H0_eci_pre_events),1e-10);

out.t=t;
out.x=x;
out.omega=omega;
out.q=q;
out.h=h;
out.mode=mode;
out.rateDeg=rate;
out.errorDeg=errorDeg;
out.Heci=Heci;
out.H0eci_pre_events=H0_eci_pre_events;
out.HbalanceRel=H_err/H_scale;
out.tauExternal=torqueExternal;
out.tauMag=torqueMag;
out.u=torqueMotor;
out.m=magDipole;
out.dipoleSaturated=dipoleSaturated;
out.motorSaturated=motorSaturated;
out.qNormError=qNormError;
out.r_eci_m=r;
out.B_eci_T=B;
out.switchIndex=find(~strcmp(mode,'detumble'),1,'first');
out.switchTime_s=switchTime_s;
out.switchReason=switchReason;
out.modePolicy=cfg.modePolicy;
out.events=cfg.events;

assert(max(qNormError)<1e-9,'Quaternion normalization failed.');
% Check each actuator against its own limit, including asymmetric setups.
assert(all(all(abs(h) <= p.hMax' + 1e-10)), ...
    'A wheel exceeded its own nominal momentum capacity.');
assert(all(all(abs(torqueMotor) <= p.uMax' + 1e-10)), ...
    'A wheel motor exceeded its own commandable torque.');
assert(all(all(abs(magDipole) <= p.mMax' + 1e-10)), ...
    'A magnetic torque rod exceeded its own rated dipole.');
assert(max(out.HbalanceRel)<5e-4, ...
    'External torque / inertial angular-momentum balance failed.');
end

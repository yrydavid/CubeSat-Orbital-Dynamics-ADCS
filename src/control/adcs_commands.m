function a=adcs_commands(mode,w,q,h,B_body,p,cfg)
%ADCS_COMMANDS Three MTQs + three RW motor torques.
% mode: 'detumble' or 'point_dump' or 'point_no_dump'.
% m is the magnetic dipole vector in BODY [A*m^2].
% u=hdot is the motor torque on RW rotors [N*m].

if strcmp(mode,'detumble')
    % Ideal rotation-only B-dot. Requires perfect body angular rate.
    mRequested=p.kBdot*cross(w,B_body);
    uRequested=zeros(3,1);
    phi=zeros(3,1);
else
    if strcmp(mode,'point_dump')
        % External moment deliberately opposes RW stored momentum.
        tauDumpRequested=-p.kDump*h;
        mRequested=cross(B_body,tauDumpRequested)/dot(B_body,B_body);
    elseif strcmp(mode,'point_no_dump')
        mRequested=zeros(3,1);
    else
        error('Unknown ADCS mode: %s',mode);
    end
    [phi,~]=attitude_error(q,cfg.qTarget);
    tauPD=p.Kp.*phi-p.Kd.*w;  % desired BODY moment
    % In the unsaturated, disturbance-free model, command the wheel
    % torque so J*wDot = tauPD. The rotor motor torque is internal.
    % Compensate magnetic body torque AND gyroscopic coupling.
    mApplied=max(-p.mMax,min(p.mMax,mRequested));
    tauMag=cross(mApplied,B_body);
    gyro=cross(w,p.J*w+h);
    uRequested=tauMag-tauPD-gyro;
end
mApplied=max(-p.mMax,min(p.mMax,mRequested));
tauMag=cross(mApplied,B_body);
uApplied=max(-p.uMax,min(p.uMax,uRequested));
% No outward torque at the wheel momentum capacity.
outHigh=(h>=p.hMax-1e-12)&(uApplied>0);
outLow=(h<=-p.hMax+1e-12)&(uApplied<0);
uApplied(outHigh|outLow)=0;

a.m=mApplied;
a.u=uApplied;
a.tauMag=tauMag;
a.phi=phi;
a.dipoleSaturated=any(abs(mRequested)>p.mMax);
a.motorSaturated=any(abs(uRequested)>p.uMax);
a.wheelAtCapacity=any(abs(h)>=p.hMax-1e-10);
end

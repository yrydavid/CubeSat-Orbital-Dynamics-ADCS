function a=adcs_commands(mode,w,q,h,B_body,p,cfg)
%ADCS_COMMANDS Magnetic dipole and reaction-wheel motor commands.
% mode: 'detumble', 'point_dump', or 'point_no_dump'.
% m is the magnetic dipole vector in BODY [A*m^2].
% u=hdot is the motor torque on RW rotors [N*m].

isDetumbling=strcmp(mode,'detumble');
if isDetumbling
    % Rate-aided rotational B-dot approximation (ideal angular-rate input).
    mRequested=p.kBdot*cross(w,B_body);
    phi=zeros(3,1);
else
    if strcmp(mode,'point_dump')
        % Magnetic torque request for reduction of stored wheel momentum.
        tauDumpRequested=-p.kDump*h;
        Bsq=dot(B_body,B_body);
        if Bsq<=1e-18
            % Near-zero field: no useful magnetic control authority.
            mRequested=zeros(3,1);
        else
            mRequested=cross(B_body,tauDumpRequested)/Bsq;
        end
    elseif strcmp(mode,'point_no_dump')
        mRequested=zeros(3,1);
    else
        error('Unknown ADCS mode: %s',mode);
    end
    [phi,~]=attitude_error(q,cfg.qTarget);
    tauPD=p.Kp.*phi-p.Kd.*w; % desired BODY moment
end

% Apply magnetic dipole limits once for both operating modes.
mApplied=max(-p.mMax,min(p.mMax,mRequested));
tauMag=cross(mApplied,B_body);

if isDetumbling
    uRequested=zeros(3,1);
else
    % In the ideal unsaturated model, wheel feedforward compensates the
    % modeled magnetic torque and rigid-body gyroscopic coupling.
    gyro=cross(w,p.J*w+h);
    uRequested=tauMag-tauPD-gyro;
end
uApplied=max(-p.uMax,min(p.uMax,uRequested));
% Prevent outward motor torque when a wheel reaches its momentum limit.
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

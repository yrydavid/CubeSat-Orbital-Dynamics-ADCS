function [dy,diag] = adcs_rhs(x,r_eci_m,B_eci_T,mode,p,cfg,eventTorque)
%ADCS_RHS Coupled 3D rigid body + 3 reaction wheels + magnetic torquers.
% State [omega_body(3);q_body_to_eci(4);h_wheel(3);integral_tau_eci(3)].
% J*omega_dot=tau_external-u-cross(omega,J*omega+h)
% h_dot=u. All torques in BODY [N*m].
% Optional eventTorque.body and eventTorque.eci are external finite torques.
if nargin<7 || isempty(eventTorque)
    eventTorque=struct('body',zeros(3,1),'eci',zeros(3,1));
end
w=x(1:3); q=x(4:7); h=x(8:10);
q=q/norm(q);
C=quat_to_dcm_b2i(q);
Bb=C'*B_eci_T;
rhatBody=C'*(r_eci_m/norm(r_eci_m));

% Gravity-gradient: Celani (2021), Eq. (7); residual dipole: Sec. 4.
tauGG=3*(cfg.mu_km*1e9)/norm(r_eci_m)^3 * ...
    cross(rhatBody,p.J*rhatBody);
tauResidual=cross(p.mResidual,Bb);

% User-defined finite torque, correctly transformed to BODY.
tauCustom=eventTorque.body+C'*eventTorque.eci;

a=adcs_commands(mode,w,q,h,Bb,p,cfg);
tauExternal=tauGG+tauResidual+a.tauMag+tauCustom;
wDot=p.J\(tauExternal-a.u-cross(w,p.J*w+h));
qDot=0.5*hamilton_mul(q,[0;w]);
hDot=a.u;
intHDot=C*tauExternal;
dy=[wDot;qDot;hDot;intHDot];
if nargout>1
    diag=a;
    diag.tauGG=tauGG;
    diag.tauResidual=tauResidual;
    diag.tauCustom=tauCustom;
    diag.tauExternal=tauExternal;
    diag.Bbody=Bb;
end
end

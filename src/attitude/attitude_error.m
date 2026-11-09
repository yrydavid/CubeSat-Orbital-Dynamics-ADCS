function [phi,theta] = attitude_error(qCurrent,qDesired)
%ATTITUDE_ERROR Shortest rotation from actual to target, in BODY axes.
qc=[qCurrent(1);-qCurrent(2:4)];
qe=hamilton_mul(qc,qDesired); qe=qe/norm(qe);
if qe(1)<0, qe=-qe; end
v=qe(2:4); nv=norm(v);
theta=2*atan2(nv,qe(1));
if nv>1e-12
    phi=theta*v/nv;
else
    phi=2*v;
end
end

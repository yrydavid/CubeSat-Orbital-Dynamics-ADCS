function C=quat_to_dcm_b2i(q)
%QUAT_TO_DCM_B2I Active BODY->ECI rotation, scalar-first Hamilton.
q=q/norm(q); s=q(1); v=q(2:4);
vx=[0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
C=(s^2-dot(v,v))*eye(3)+2*(v*v')+2*s*vx;
end

function dy = j2_rhs(~,y,mu,Re,J2)
%J2_RHS State [r_km;v_km_s], acceleration in km/s^2.
r=y(1:3); v=y(4:6); rho=norm(r);
zr2=(r(3)/rho)^2;
a0=-mu*r/rho^3;
f=1.5*J2*mu*Re^2/rho^5;
a2=f*[r(1)*(5*zr2-1);r(2)*(5*zr2-1);r(3)*(5*zr2-3)];
dy=[v;a0+a2];
end

function qp=hamilton_mul(q,p)
%HAMILTON_MUL Scalar-first Hamilton quaternion product.
qv=q(2:4); pv=p(2:4);
qp=[q(1)*p(1)-dot(qv,pv); ...
    q(1)*pv+p(1)*qv+cross(qv,pv)];
end

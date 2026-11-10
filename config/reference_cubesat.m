function p = reference_cubesat()
%REFERENCE_CUBESAT Parameters for the virtual 3U reference case.
% All units SI unless noted. See docs/sources.md.
%
% This is NOT a model of the flown Tigrisat hardware configuration.
p.J = diag([0.0409 0.0409 0.0065]);      % kg*m^2, Celani Eq. (31)
p.hMax = 5.8e-4 * ones(3,1);         % N*m*s, RW1 Type A nominal storage
p.uMax = 23e-6 * ones(3,1);          % N*m, RW1 Type A max commandable
p.mMax = 0.4 * ones(3,1);            % A*m^2, CR0004 dipole at 5 V (ideal bidirectional envelope)
p.mResidual = [0;0;3e-4];            % A*m^2, Celani simulation assumption

% Gains: ORIGINAL design choices, not manufacturer or flight values.
p.kBdot = 3e5;                       % A*m^2*s/T, ideal gyro-assisted
p.kDump = 8e-4;                      % 1/s, wheel momentum unloading
p.Kp = [1.2e-5;1.2e-5;5e-6];         % N*m/rad
p.Kd = [1.2e-3;1.2e-3;3e-4];         % N*m*s/rad

% Three RW1 Type A and three CR0004 orthogonal actuators are ASSUMED.
% Locked-assembly inertia assumption: wheel relative momentum is added
% separately; rotor locked inertia is NOT added to p.J again.
assert(all(eig(p.J)>0) && norm(p.J-p.J','fro')<1e-12);
assert(all(p.hMax>0));
assert(all(p.uMax>0) && all(p.mMax>0));
end

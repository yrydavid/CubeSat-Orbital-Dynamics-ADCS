function a = custom_torque_at_midpoint(events,midpoint_s)
%CUSTOM_TORQUE_AT_MIDPOINT Sum active finite disturbance torques.
% Samples at RK4 step midpoint, held constant over that full time step.
% BODY torques are fixed to the satellite; ECI torques are inertially fixed.
% The BODY rotation of an ECI torque is handled within adcs_rhs.
a.body=zeros(3,1);
a.eci=zeros(3,1);
for j=1:numel(events)
    e=events(j);
    if strcmpi(e.type,'torque') && midpoint_s>=e.time_s && ...
            midpoint_s<e.time_s+e.duration_s
        if strcmpi(e.frame,'BODY')
            a.body=a.body+e.vector;
        else
            a.eci=a.eci+e.vector;
        end
    end
end
end

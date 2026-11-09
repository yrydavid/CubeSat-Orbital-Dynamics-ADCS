function [x,appliedImpulseECI] = apply_impulses_at_time(x,events,t_s,p)
%APPLY_IMPULSES_AT_TIME Instantaneous jumps at precisely aligned grid times.
% Angular impulse changes body rates but not the quaternion or wheel speeds.
% The added ECI angular impulse must appear in the momentum-balance state.
appliedImpulseECI=zeros(3,1);
if isempty(events)
    return;
end
C=quat_to_dcm_b2i(x(4:7));
for j=1:numel(events)
    e=events(j);
    if strcmpi(e.type,'impulse') && abs(e.time_s-t_s)<1e-8
        if strcmpi(e.frame,'BODY')
            impulseBody=e.vector;
            impulseECI=C*impulseBody;
        else
            impulseECI=e.vector;
            impulseBody=C'*impulseECI;
        end
        x(1:3)=x(1:3)+p.J\impulseBody;
        x(11:13)=x(11:13)+impulseECI;
        appliedImpulseECI=appliedImpulseECI+impulseECI;
    end
end
end

function metrics=event_recovery_metrics(base,pert,events,threshold_deg,dwell_s)
%EVENT_RECOVERY_METRICS Recovery relative to an unperturbed baseline.
% Relative attitude separation uses the full quaternion rotation, NOT the
% difference between two scalar pointing errors.
% Metrics are computed up to the NEXT event. When event activity overlaps,
% individual event attribution is undefined and is returned as NaN.
% Nonoverlapping events still reflect the cumulative simulated trajectory.
assert(isequal(size(base.q),size(pert.q)) && ...
    isequal(base.t,pert.t),'Baseline and event runs need a shared grid.');
t=base.t;
dt=t(2)-t(1);
N=numel(t);
sep=zeros(N,1);
rateDifference_deg_s=zeros(N,1);
for k=1:N
    qb=base.q(k,:)'; qe=pert.q(k,:)';
    qc=[qb(1);-qb(2:4)];
    qr=hamilton_mul(qc,qe);
    qr=qr/norm(qr);
    sep(k)=rad2deg(2*atan2(norm(qr(2:4)),abs(qr(1))));

    % Physical angular-rate comparison must use ONE COMMON frame.
    % The two omega rows are expressed in DIFFERENT moving BODY frames
    % when the event perturbs the orientation. Rotate both into ECI.
    wb_eci=quat_to_dcm_b2i(qb)*base.omega(k,:)';
    we_eci=quat_to_dcm_b2i(qe)*pert.omega(k,:)';
    rateDifference_deg_s(k)=rad2deg(norm(we_eci-wb_eci));
end
metrics.separation_deg=sep;
metrics.angular_rate_difference_deg_s=rateDifference_deg_s;
% Componentwise wheel telemetry is compared in the corresponding
% hardware-labeled BODY channels (not as an inertial-vector difference).
metrics.wheel_difference_Nms=vecnorm(pert.h-base.h,2,2);
metrics.event=struct([]);
% Events can be entered in any order in the configuration.
if ~isempty(events)
    [~,order]=sort([events.time_s]);
    events=events(order);
end
for j=1:numel(events)
    e=events(j);
    if strcmpi(e.type,'torque')
        tEnd=e.time_s+e.duration_s;
    else
        tEnd=e.time_s;
    end
    if j<numel(events)
        tNext=min([events(j+1:end).time_s]);
    else
        tNext=t(end)+dt;
    end
    segment=(t>=e.time_s & t<tNext);
    recovery=(t>=tEnd & t<tNext);
    % Check all other event intervals, including an earlier torque still
    % active when this event starts. This also catches simultaneous events
    % and impulses occurring during finite torques.
    hasOverlap=false;
    for other=1:numel(events)
        if other==j
            continue;
        end
        if events_overlap(e,events(other))
            hasOverlap=true;
            break;
        end
    end
    ids=find(recovery);
    recoveryTime_s=NaN;
    windowLength=round(dwell_s/dt)+1;
    for k=1:max(0,numel(ids)-windowLength+1)
        window=ids(k:k+windowLength-1);
        if all(sep(window)<threshold_deg)
            recoveryTime_s=t(window(1))-tEnd;
            break;
        end
    end
    metrics.event(j).name=upper(e.type);
    metrics.event(j).start_s=e.time_s;
    metrics.event(j).end_s=tEnd;
    metrics.event(j).has_overlapping_event=hasOverlap;
    if hasOverlap
        metrics.event(j).max_attitude_separation_deg=NaN;
        metrics.event(j).max_omega_difference_deg_s=NaN;
        metrics.event(j).max_wheel_difference_Nms=NaN;
        metrics.event(j).recoveryTime_s=NaN;
        warning('CubeSat:OverlappingEvents', ...
            'Event at %.1f s overlaps another event; per-event metrics are undefined.',e.time_s);
    else
        metrics.event(j).max_attitude_separation_deg=max(sep(segment));
        metrics.event(j).max_omega_difference_deg_s= ...
            max(metrics.angular_rate_difference_deg_s(segment));
        metrics.event(j).max_wheel_difference_Nms= ...
            max(metrics.wheel_difference_Nms(segment));
        metrics.event(j).recoveryTime_s=recoveryTime_s;
    end
end
end

function tf=events_overlap(a,b)
%EVENTS_OVERLAP Intervals are [start,end); impulses are time points.
aTorque=strcmpi(a.type,'torque');
bTorque=strcmpi(b.type,'torque');
if aTorque && bTorque
    tf=max(a.time_s,b.time_s)< ...
        min(a.time_s+a.duration_s,b.time_s+b.duration_s);
elseif aTorque
    tf=b.time_s>=a.time_s && b.time_s<a.time_s+a.duration_s;
elseif bTorque
    tf=a.time_s>=b.time_s && a.time_s<b.time_s+b.duration_s;
else
    tf=a.time_s==b.time_s;
end
end

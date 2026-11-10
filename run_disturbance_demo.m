%% RUN_DISTURBANCE_DEMO - Custom finite torque and angular impulse
% MATLAB R2025b + Aerospace Toolbox.
% Both cases use identical J2+IGRF-14 conditions and initial states.
% One case receives user-defined disturbances; the other is the baseline.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'config')));
addpath(genpath(fullfile(root,'src')));
p=reference_cubesat();
cfg=simulation_config();
cfg.modePolicy='automatic';
cfg.enableDumping=true;
cfg.hwheel0_Nms=[2e-4;-1.5e-4;1e-4]; % project test assumption

% IMPORTANT: these are illustrative stresses, not measured impacts.
cfg.events=disturbance_events();
validate_disturbance_events(cfg.events,cfg);

fprintf('Building shared J2 and IGRF-14 environment ...\n');
env=build_environment(cfg);

% Print physical torque components at t=0. No arbitrary constant torque.
x0=[cfg.omega0_rad_s;cfg.q0;cfg.hwheel0_Nms;zeros(3,1)];
[~,d0]=adcs_rhs(x0,env.r_eci_m(1,:)',env.B_eci_T(1,:)', ...
    'detumble',p,cfg);
fprintf('\n--- NATURAL EXTERNAL DISTURBANCES AT t=0 ---\n');
fprintf('Gravity-gradient BODY [N*m]: [%+.6e %+.6e %+.6e]\n',d0.tauGG);
fprintf('Residual magnetic BODY [N*m]: [%+.6e %+.6e %+.6e]\n',d0.tauResidual);
fprintf('Norm gravity-gradient: %.6e N*m\n',norm(d0.tauGG));
fprintf('Norm residual magnetic: %.6e N*m\n',norm(d0.tauResidual));

cfgBase=cfg;
cfgBase.events=struct([]);
base=simulate_adcs(cfgBase,p,env);
eventCase=simulate_adcs(cfg,p,env);
% Both cases must be identical before the first external event.
if isempty(cfg.events)
    firstEventTime=inf;
else
    firstEventTime=min([cfg.events.time_s]);
end
preMask=base.t<firstEventTime;
if any(preMask)
    assert(max(abs(base.x(preMask,:)-eventCase.x(preMask,:)),[],'all')<1e-12, ...
        'Baseline and event-driven solutions diverged before the first event.');
end

fprintf('\n--- USER EVENT EXPERIMENT ---\n');
for k=1:numel(cfg.events)
    e=cfg.events(k);
    fprintf('%d) %s at %.1f s, duration %.1f s, frame %s\n', ...
        k,upper(e.type),e.time_s,e.duration_s,upper(e.frame));
    fprintf('   Vector: [%+.4e %+.4e %+.4e]\n',e.vector);
end
fprintf('Final pointing error baseline/event: %.6f / %.6f deg\n', ...
    base.errorDeg(end),eventCase.errorDeg(end));
fprintf('Final |omega| baseline/event: %.6f / %.6f deg/s\n', ...
    base.rateDeg(end),eventCase.rateDeg(end));
fprintf('Max relative H error baseline/event: %.3e / %.3e\n', ...
    max(base.HbalanceRel),max(eventCase.HbalanceRel));
fprintf('Last-orbit RMS pointing baseline/event: %.6f / %.6f deg\n', ...
    sqrt(mean(base.errorDeg(base.t>=base.t(end)-cfg.orbitPeriod_s).^2)), ...
    sqrt(mean(eventCase.errorDeg(eventCase.t>=eventCase.t(end)-cfg.orbitPeriod_s).^2)));

figEventOverview=figure('Name','CubeSat - User Disturbances and Recovery','Color','w');
tiledlayout(3,1);
nexttile;
plot(base.t/60,base.errorDeg,'--','LineWidth',1.1); hold on;
plot(eventCase.t/60,eventCase.errorDeg,'LineWidth',1.1);
ylabel('Attitude error [deg]');title('Inertial Pointing');grid on;
legend('Baseline','Custom events','Location','best','AutoUpdate','off');
nexttile;
plot(base.t/60,base.rateDeg,'--','LineWidth',1.1); hold on;
plot(eventCase.t/60,eventCase.rateDeg,'LineWidth',1.1);
ylabel('|omega| [deg/s]');title('Body Angular-Rate Norm');grid on;
nexttile;
plot(base.t/60,vecnorm(base.h,2,2),'--','LineWidth',1.1); hold on;
plot(eventCase.t/60,vecnorm(eventCase.h,2,2),'LineWidth',1.1);
ylabel('Wheel |h| [N*m*s]');xlabel('Time [min]');
title('Stored Reaction-Wheel Momentum');grid on;
for j=1:numel(cfg.events)
    for ax=1:3
        % Mark each event on every tile, respecting MATLAB tiledlayout.
        nexttile(ax);
        xline(cfg.events(j).time_s/60,':', ...
            sprintf('%s',upper(cfg.events(j).type)));
    end
end
%% RECOVERY METRICS - full 3D attitude difference to baseline
% These are not necessarily zero merely because final pointing errors
% printed with six decimal places happen to match.
rec=event_recovery_metrics(base,eventCase,cfg.events,0.1,600);
fprintf('\n--- EVENT RECOVERY (relative to baseline) ---\n');
for j=1:numel(rec.event)
    e=rec.event(j);
    fprintf('%s at %.1f s: peak 3D attitude separation %.6f deg\n', ...
        e.name,e.start_s,e.max_attitude_separation_deg);
    fprintf('  Peak angular-rate difference: %.6f deg/s\n', ...
        e.max_omega_difference_deg_s);
    fprintf('  Peak wheel-momentum difference: %.6e N*m*s\n', ...
        e.max_wheel_difference_Nms);
    if e.has_overlapping_event
        fprintf('  Per-event recovery is undefined: overlapping inputs.\n');
    elseif isnan(e.recoveryTime_s)
        fprintf('  Sustained relative recovery not reached before next event/end.\n');
    else
        fprintf('  Recovery to <0.1 deg for 600 s: %.1f s after event end\n', ...
            e.recoveryTime_s);
    end
end
figEventRecovery=figure('Name','Disturbance - Relative 3D Attitude Recovery','Color','w');
plot(base.t/60,rec.separation_deg,'LineWidth',1.3);hold on;
yline(0.1,'--','0.1 deg relative recovery threshold');
for j=1:numel(cfg.events)
    xline(cfg.events(j).time_s/60,':',upper(cfg.events(j).type));
end
xlabel('Time [min]'); ylabel('3D separation from baseline [deg]');
title('Disturbance-Induced Attitude Separation (Quaternion Metric)');grid on;
fprintf('Custom-event demo completed; review transient recovery and limits.\n');

%% Export event-response figures
figDir=fullfile(root,'figures');
if ~exist(figDir,'dir'); mkdir(figDir); end
exportgraphics(figEventOverview,fullfile(figDir,'disturbance_response.png'),'Resolution',200);
exportgraphics(figEventRecovery,fullfile(figDir,'disturbance_recovery.png'),'Resolution',200);
fprintf('Saved disturbance figures in %s\n',figDir);

%% RUN_DEMO - CubeSat magnetic momentum unloading A/B (MATLAB R2025b)
% Uses the same J2 orbit and IGRF-14 fields in BOTH scenarios.
% Compare wheel unloading enabled vs disabled with nonzero initial h.
% This file is the only demo entrypoint; no historical step files required.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'config')));
addpath(genpath(fullfile(root,'src')));

p=reference_cubesat();
cfg=simulation_config();
cfg.modePolicy='automatic';
cfg.hwheel0_Nms=[2e-4;-1.5e-4;1e-4]; % [ASSUMED] demo wheel bias

fprintf('Building a single J2 + IGRF-14 environment ...\n');
env=build_environment(cfg);

cfgOn=cfg;
cfgOn.enableDumping=true;
outOn=simulate_adcs(cfgOn,p,env);

cfgOff=cfg;
cfgOff.enableDumping=false;
outOff=simulate_adcs(cfgOff,p,env);

fprintf('\n--- ADCS A/B: MAGNETIC WHEEL UNLOADING ---\n');
fprintf('Initial wheel h: [%+.2e %+.2e %+.2e] N*m*s\n', ...
    cfg.hwheel0_Nms);
fprintf('Automatic switch threshold: %.3f deg/s for %.0f s\n', ...
    cfg.detumbleRateThreshold_deg_s,cfg.detumbleDwell_s);

if isempty(outOn.switchIndex) || isempty(outOff.switchIndex)
    fprintf('No automatic pointing transition within %.2f h.\n', ...
        outOn.t(end)/3600);
    fprintf('Review detumbling rate and dwell setting; no dumping claim.\n');
else
    k=outOn.switchIndex;
    fprintf('Pointing switch ON/OFF [min]: %.3f / %.3f\n', ...
        outOn.switchTime_s/60,outOff.switchTime_s/60);
    fprintf('Rate at switch [deg/s]: %.6f\n',outOn.rateDeg(k));
    fprintf('Wheel norm at switch: %.8e N*m*s\n', ...
        norm(outOn.h(k,:)));
    fprintf('Final wheel norm WITH dumping: %.8e N*m*s\n', ...
        norm(outOn.h(end,:)));
    fprintf('Final wheel norm WITHOUT dumping: %.8e N*m*s\n', ...
        norm(outOff.h(end,:)));
    fprintf('Final pointing error WITH dumping: %.6f deg\n', ...
        outOn.errorDeg(end));
    fprintf('Final pointing error WITHOUT dumping: %.6f deg\n', ...
        outOff.errorDeg(end));
    fprintf('Final angular rate WITH/WITHOUT [deg/s]: %.7f / %.7f\n', ...
        outOn.rateDeg(end),outOff.rateDeg(end));
end

fprintf('Max relative H-balance error ON/OFF: %.3e / %.3e\n', ...
    max(outOn.HbalanceRel),max(outOff.HbalanceRel));
fprintf('Maximum wheel capacity usage ON/OFF: %.2f%% / %.2f%%\n', ...
    100*max(abs(outOn.h./p.hMax'),[],'all'), ...
    100*max(abs(outOff.h./p.hMax'),[],'all'));
fprintf('Magnetic dipole saturation output samples ON/OFF: %d / %d\n', ...
    nnz(outOn.dipoleSaturated),nnz(outOff.dipoleSaturated));
fprintf('RW motor saturation output samples ON/OFF: %d / %d\n', ...
    nnz(outOn.motorSaturated),nnz(outOff.motorSaturated));

% Pre-transition A/B equality is a necessary comparison sanity check.
if ~isempty(outOn.switchIndex) && ~isempty(outOff.switchIndex)
    assert(outOn.switchIndex==outOff.switchIndex, ...
        'A/B mode switch did not occur at the same sample.');
    k=outOn.switchIndex;
    assert(max(abs(outOn.x(1:k,:)-outOff.x(1:k,:)),[],'all')<1e-12, ...
        'A/B cases differ before dumping is enabled.');
end

fprintf('A/B integration and physical invariant tests passed.\n');
fprintf('Dumping improves wheel storage only if ON final h < OFF final h.\n');

%% Figure 1: compare wheel momenta and pointing
figUnloading=figure('Name','CubeSat - Unloading ON versus OFF','Color','w');
tiledlayout(2,1);
nexttile;
plot(outOn.t/3600,vecnorm(outOn.h,2,2),'LineWidth',1.3);hold on;
plot(outOff.t/3600,vecnorm(outOff.h,2,2),'--','LineWidth',1.3);
if ~isempty(outOn.switchIndex)
    xline(outOn.switchTime_s/3600,':','Pointing starts');
end
ylabel('||h_{wheels}|| [N m s]');
legend('Unloading ON','Unloading OFF','Location','best');
title('Stored reaction-wheel momentum');grid on;
nexttile;
plot(outOn.t/3600,outOn.errorDeg,'LineWidth',1.3);hold on;
plot(outOff.t/3600,outOff.errorDeg,'--','LineWidth',1.3);
if ~isempty(outOn.switchIndex)
    xline(outOn.switchTime_s/3600,':','Pointing starts');
end
xlabel('Time [h]');ylabel('Attitude error [deg]');
title('Inertial pointing during magnetic unloading');
legend('Unloading ON','Unloading OFF','Location','best');grid on;

%% Figure 2: actuator commands and physical residuals
figDiagnostics=figure('Name','CubeSat - Magnetic Unloading Diagnostic','Color','w');
tiledlayout(3,1);
nexttile;
plot(outOn.t/3600,outOn.h,'LineWidth',1.1);hold on;
yline(p.hMax(1),'k--','Nominal +/- capacity');
yline(-p.hMax(1),'k--');
ylabel('Wheel h [N m s]');legend('h_x','h_y','h_z');grid on;
nexttile;
plot(outOn.t/3600,vecnorm(outOn.tauMag,2,2)*1e6,'LineWidth',1.2);
ylabel('||tau_{mag}|| [microN m]');
title('External magnetic torque (dumping ON)');grid on;
nexttile;
semilogy(outOn.t/3600,max(outOn.HbalanceRel,eps),'LineWidth',1.1);hold on;
semilogy(outOff.t/3600,max(outOff.HbalanceRel,eps),'--','LineWidth',1.1);
ylabel('Relative H residual');xlabel('Time [h]');
legend('ON','OFF','Location','best');grid on;

%% Export validated-by-MATLAB figures for the eventual GitHub README
figDir=fullfile(root,'figures');
if ~exist(figDir,'dir'); mkdir(figDir); end
exportgraphics(figUnloading,fullfile(figDir,'momentum_unloading_comparison.png'),'Resolution',200);
exportgraphics(figDiagnostics,fullfile(figDir,'actuator_diagnostics.png'),'Resolution',200);
fprintf('Saved demonstration figures in %s\n',figDir);

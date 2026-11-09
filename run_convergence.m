%% RUN_CONVERGENCE - Integration and environment-grid sensitivity
% MATLAB R2025b + Aerospace Toolbox.
% This is a numerical verification, not a new physical subsystem.
% No user-defined events, so differences isolate integration and IGRF grids.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'config')));
addpath(genpath(fullfile(root,'src')));
p=reference_cubesat();
cfgBase=simulation_config();
cfgBase.modePolicy='automatic';
cfgBase.enableDumping=true;
cfgBase.hwheel0_Nms=[2e-4;-1.5e-4;1e-4];
cfgBase.events=struct([]);

% Use a common final time exactly divisible by every comparison grid.
commonFinalTime_s=floor(cfgBase.numOrbits*cfgBase.orbitPeriod_s/10)*10;
% Columns: attitude integration dt, environment/IGRF dt [seconds].
grids=[2,10;1,5;0.5,2.5];
numCases=size(grids,1);
finalError=zeros(numCases,1);
finalRate=zeros(numCases,1);
finalWheel=zeros(numCases,1);
modeTime=zeros(numCases,1);
maxBalance=zeros(numCases,1);
capacityCount=zeros(numCases,1);

for j=1:numCases
    cfg=cfgBase;
    cfg.dt_s=grids(j,1);
    cfg.envDt_s=grids(j,2);
    cfg.finalTime_s=commonFinalTime_s;
    cfg.detumbleEnd_s=round(cfg.orbitPeriod_s/cfg.dt_s)*cfg.dt_s;
    fprintf('\nRunning dt=%.1f s, environment dt=%.1f s ...\n', ...
        cfg.dt_s,cfg.envDt_s);
    env=build_environment(cfg);
    s=simulate_adcs(cfg,p,env);
    finalError(j)=s.errorDeg(end);
    finalRate(j)=s.rateDeg(end);
    finalWheel(j)=norm(s.h(end,:));
    modeTime(j)=s.switchTime_s;
    maxBalance(j)=max(s.HbalanceRel);
    capacityCount(j)=nnz(any(abs(s.h)>=p.hMax'-1e-10,2));
end
fprintf('\n--- GRID CONVERGENCE REPORT ---\n');
fprintf(' dt   envDt   pointErr[deg]  rate[deg/s]    |h|[Nms]       Hbalance   switch[min]  capSamples\n');
for j=1:numCases
    fprintf('%4.1f %6.1f %15.8f %12.8f %13.6e %12.3e %11.3f %10d\n', ...
        grids(j,1),grids(j,2),finalError(j),finalRate(j), ...
        finalWheel(j),maxBalance(j),modeTime(j)/60,capacityCount(j));
end
fprintf('\nDifference relative to finest tested grid:\n');
for j=1:numCases-1
    fprintf('dt=%.1f: delta error %.6g deg; delta |h| %.6g Nms; delta switch %.3f s\n', ...
        grids(j,1),abs(finalError(j)-finalError(end)), ...
        abs(finalWheel(j)-finalWheel(end)), ...
        abs(modeTime(j)-modeTime(end)));
end
assert(all(isfinite(finalError)) && all(isfinite(finalWheel)) && ...
    all(maxBalance<5e-4),'Convergence run has invalid results.');
% Scenario-level convergence gates, not universal error guarantees.
maxPointDifference_deg=max(abs(finalError(1:end-1)-finalError(end)));
maxWheelDifference_Nms=max(abs(finalWheel(1:end-1)-finalWheel(end)));
assert(maxPointDifference_deg<1e-3, ...
    'Final pointing error changed by >= 0.001 deg under grid refinement.');
assert(maxWheelDifference_Nms<1e-6, ...
    'Final wheel-momentum norm is grid-sensitive by >= 1e-6 N*m*s.');
fprintf(['Grid convergence criteria passed for this scenario: ', ...
    'max delta pointing %.3e deg, max delta |h| %.3e N*m*s.\n'], ...
    maxPointDifference_deg,maxWheelDifference_Nms);

%% Visual summary of error reduction across tested grids
figConvergence=figure('Name','CubeSat - Grid Convergence','Color','w');
loglog(grids(:,1),maxBalance,'-o','LineWidth',1.3,'MarkerSize',6);
set(gca,'XDir','reverse');
grid on; xlabel('Attitude integration step [s]');
ylabel('Max relative inertial-momentum balance error');
title('Time-step and IGRF Sampling Refinement');
figDir=fullfile(root,'figures');
if ~exist(figDir,'dir'); mkdir(figDir); end
exportgraphics(figConvergence,fullfile(figDir,'grid_convergence.png'),'Resolution',200);
fprintf('Saved convergence figure in %s\n',figDir);

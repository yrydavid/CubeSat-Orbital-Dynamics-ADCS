%% RUN_ENVIRONMENT_DEMO - J2 orbit and IGRF-14 visualizations
% MATLAB R2025b + Aerospace Toolbox.
% Uses the SAME canonical environment functions as run_demo.m.
% No legacy step14 scripts or assumed magnetic field values required.
clear; clc; close all;
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'config')));
addpath(genpath(fullfile(root,'src')));
cfg = simulation_config();
env = build_environment(cfg);

% Convert position to km and field strength to microtesla for the figures.
r_km = env.r_eci_m / 1e3;
t_h = env.t / 3600;
B_uT = env.field_uT;
Re_km = cfg.Re_km;

% Figure 1: orbit trajectory in ECI colored by real IGRF-14 field magnitude.
fig1 = figure('Name','Orbital Environment - J2 and IGRF-14','Color','w');
ax = axes(fig1);
hold(ax,'on');
[sx,sy,sz] = sphere(42);
surf(ax,Re_km*sx,Re_km*sy,Re_km*sz, ...
    'FaceColor',[0.75 0.84 0.94], ...
    'FaceAlpha',0.40,'EdgeColor','none', ...
    'HandleVisibility','off');
scatter3(ax,r_km(:,1),r_km(:,2),r_km(:,3), ...
    14,B_uT,'filled');
plot3(ax,r_km(1,1),r_km(1,2),r_km(1,3), ...
    'ko','MarkerFaceColor','w','MarkerSize',7);
cb = colorbar(ax);
cb.Label.String = '|B| [microT]';
clim(ax,[min(B_uT),max(B_uT)]);  % Only the magnetic colors, not sphere z.
axis(ax,'equal');
grid(ax,'on');
view(ax,40,25);
xlabel(ax,'ECI X [km]');
ylabel(ax,'ECI Y [km]');
zlabel(ax,'ECI Z [km]');
title(ax,'J2 Orbit Colored by IGRF-14 Magnetic Field Strength');

% Figure 2: three independent environmental observables.
fig2 = figure('Name','Orbital Environment - Time History','Color','w');
tiledlayout(fig2,3,1,'TileSpacing','compact');
nexttile;
plot(t_h,B_uT,'LineWidth',1.15);
ylabel('|B| [microT]');
title('IGRF-14 Magnetic Field Along the Orbit');
grid on;
nexttile;
plot(t_h,env.latitude_deg,'LineWidth',1.15);
ylabel('Latitude [deg]');
title('Geodetic Latitude (WGS84)');
grid on;
nexttile;
plot(t_h,env.altitude_m/1000,'LineWidth',1.15);
ylabel('Altitude [km]');
xlabel('Time [h]');
title('Geodetic Altitude (WGS84)');
grid on;

% Export reproducible figures for documentation / README.
figDir = fullfile(root,'figures');
if ~exist(figDir,'dir')
    mkdir(figDir);
end
exportgraphics(fig1,fullfile(figDir,'orbital_environment.png'), ...
    'Resolution',200);
exportgraphics(fig2,fullfile(figDir,'geomagnetic_time_history.png'), ...
    'Resolution',200);

fprintf('\n--- J2 + IGRF-14 VISUALIZATION ---\n');
fprintf('Orbit duration: %.3f h\n',env.t(end)/3600);
fprintf('Field magnitude: %.3f to %.3f microT\n', ...
    min(B_uT),max(B_uT));
fprintf('Altitude: %.3f to %.3f km\n', ...
    min(env.altitude_m)/1000,max(env.altitude_m)/1000);
fprintf('Exported figures to: %s\n',figDir);

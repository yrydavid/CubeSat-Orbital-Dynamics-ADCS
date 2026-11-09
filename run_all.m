%% RUN_ALL - Run the complete CubeSat reference case
% Open this repository's root folder in MATLAB and press Run (or type run_all).
% Requires MATLAB R2025b with Aerospace Toolbox.
% Tests run first; each demonstration then regenerates the PNGs in figures/.
if exist('igrfmagm','file') == 0
    error('CubeSat:MissingToolbox', ...
        'MATLAB Aerospace Toolbox with IGRF-14 (igrfmagm) is required.');
end

run_tests
run_environment_demo
run_demo
run_convergence
run_disturbance_demo

fprintf('\nCubeSat run complete. PNG figures saved in the figures folder.\n');

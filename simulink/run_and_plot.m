%% RUN_AND_PLOT
% Runs booster_return_3dof.slx and plots the logged results. The model
% is self-contained (its InitFcn derives p/bb/ld), so this just needs
% the model to be on the path.

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projRoot));

mdl = 'booster_return_3dof';
if ~bdIsLoaded(mdl)
    load_system(fullfile(projRoot, 'simulink', [mdl '.slx']));
end

simOut = sim(mdl);
simlog = simOut.get('simlog');
t = simlog.time;
data = simlog.signals.values; % columns: x y vx vy m T_cmd theta_cmd

fprintf('Simulation stopped at t = %.2f s\n', t(end));
fprintf('Final [x y vx vy m]: '); disp(data(end,1:5));

% Phase event times: boostback cutoff is the scheduled burn end; landing
% ignition is the first sample after it at/below the ignition altitude.
p  = vehicleParams();
bb = boostback_guidance([], p);
ev.tCutoff = bb.duration;
iIgn = find(t > bb.duration & data(:,2) <= p.landingIgniteAltitude, 1);
if isempty(iIgn), ev.tIgnite = NaN; else, ev.tIgnite = t(iIgn); end

plot_return_trajectory(t, data(:,1:5), data(:,6), data(:,7), p, ev, ...
                       'Booster Return - Simulink Model');

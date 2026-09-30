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

% The COESA block warns once per run that it extrapolates above 84.85 km.
% That's expected: the altitude fed to it is clamped at p.atmo.maxAltitude
% (see vehicleParams), where the extrapolated density makes drag ~0.
wState = warning('off', 'aeroblks:aeroblkcoesa:AltMsg');
simOut = sim(mdl);
warning(wState);
simlog = simOut.get('simlog');
t = simlog.time;
data = simlog.signals.values; % columns: x y vx vy m T_cmd theta_cmd

fprintf('Simulation stopped at t = %.2f s\n', t(end));
fprintf('Final [x y vx vy m]: '); disp(data(end,1:5));

% p and bb exactly as the model's InitFcn built them for this run (it
% runs in the base workspace), rather than re-solving the targeting.
p  = evalin('base', 'p');
bb = evalin('base', 'bb');
fprintf('Boostback aim bias %.0f m (%d targeting iterations)\n', bb.aimBias, bb.iterations);
fprintf('Ignition point: x = %.1f m from the landing site, speed %.0f m/s\n', ...
        data(find(t > bb.duration & data(:,2) <= p.landingIgniteAltitude, 1), 1) - p.target.x, ...
        norm(data(find(t > bb.duration & data(:,2) <= p.landingIgniteAltitude, 1), 3:4)));

% Phase event times: boostback cutoff is the scheduled burn end; landing
% ignition is the first sample after it at/below the ignition altitude;
% the entry burn is any thrust in between.
ev.tCutoff = bb.duration;
iIgn = find(t > bb.duration & data(:,2) <= p.landingIgniteAltitude, 1);
if isempty(iIgn), ev.tIgnite = NaN; else, ev.tIgnite = t(iIgn); end
entryOn = t > bb.duration & t < ev.tIgnite & data(:,6) > 0;
ev.tEntry = [];
if any(entryOn), ev.tEntry = [t(find(entryOn, 1)) t(find(entryOn, 1, 'last'))]; end

plot_return_trajectory(t, data(:,1:5), data(:,6), data(:,7), p, ev, ...
                       'Booster Return - Simulink Model');

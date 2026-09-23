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

figure('Name', 'Booster Return - Simulink Model');

subplot(2,2,1);
plot(data(:,1)/1000, data(:,2)/1000, '-o', 'LineWidth', 1.5); grid on;
xlabel('Downrange [km]'); ylabel('Altitude [km]');
title('Trajectory');

subplot(2,2,2);
plot(t, data(:,2)/1000, '-o', 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Altitude [km]');
title('Altitude vs Time');

subplot(2,2,3);
plot(t, vecnorm(data(:,3:4),2,2), '-o', 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Speed vs Time');

subplot(2,2,4);
plot(t, data(:,5), '-o', 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Mass [kg]');
title('Mass vs Time');

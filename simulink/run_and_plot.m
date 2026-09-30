%% RUN_AND_PLOT
% Runs the main simulation (booster_return_3dof.slx, via simulate_mission)
% for the nominal parameters and plots the flight. The same
% simulate_mission call is what the Monte Carlo runs for every sample.
%
% (The model can also be run from its Run button: its InitFcn then builds
% p, bb, ld itself from vehicleParams().)

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projRoot));

flight = simulate_mission(vehicleParams());
p = flight.p; bb = flight.bb;

fprintf('Simulation stopped at t = %.2f s (Simulink run %.1f s)\n', flight.t(end), flight.simTime);
fprintf('Boostback aim bias %.0f m (%d targeting iterations)\n', bb.aimBias, bb.iterations);
fprintf('Ignition point: x = %.1f m from the landing site, speed %.0f m/s\n', ...
        flight.ignition(1) - p.target.x, norm(flight.ignition(3:4)));
if flight.entrySized
    fprintf('Entry burn: %.1f-%.1f s, cutoff at %.0f kg\n', flight.ev.tEntry, flight.mCutoff);
end
fprintf('Final [x y vx vy m]: '); disp(flight.X(end, :));

plot_return_trajectory(flight.t, flight.X, flight.T, flight.theta, p, flight.ev, ...
                       'Booster Return - Simulink Model');

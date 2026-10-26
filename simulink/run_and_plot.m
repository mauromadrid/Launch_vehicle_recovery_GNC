%% RUN_AND_PLOT
% Runs the main simulation (booster_return_3dof.slx, via simulate_mission)
% for the nominal parameters, saves the flight as a record in
% results/flights, and plots it FROM THAT FILE.
%
% Running and plotting are decoupled: plot any saved flight later, without
% re-running, with
%   plot_flight_record()             % most recent record
%   plot_flight_record("nominal")    % most recent "nominal*" record
%   plot_flight_record(file, Panels=["thrust" "mach"])
%
% (The model can also be run from its Run button: its InitFcn then builds
% p, bb, ld itself from vehicleParams().)

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projRoot));

p = vehicleParams();
flight = simulate_mission(p);
fprintf('Simulation stopped at t = %.2f s (Simulink run %.1f s)\n', flight.t(end), flight.simTime);
fprintf('Ignition point: x = %.1f m from the landing site, speed %.0f m/s\n', ...
        flight.ignition(1) - p.target.x, norm(flight.ignition(3:4)));

rec = make_flight_record(flight, p, Name="nominal", Source="Simulink");
file = save_flight_record(rec);
fprintf('Saved flight record: %s\n', file);

plot_flight_record(file);

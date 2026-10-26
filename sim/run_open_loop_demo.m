%% RUN_OPEN_LOOP_DEMO
% Plain-MATLAB run of the full return sequence (the independent
% cross-check of the Simulink model):
%   - Boostback: min-fuel convex (SOCP) guidance, aimed for drag + entry
%     burn, flown as T = aCmd(t) * m(t) with its ZOH profile
%   - Coast with drag, entry burn sized at its start altitude, coast down
%     to landing-burn ignition altitude
%   - Landing: SCvx guidance re-planned in flight (fly_landing_closed_loop),
%     the same logic as the Simulink model's landing guidance block

clear; clc;
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

p = vehicleParams();

state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];

odeOpts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @touchdownEvent);

%% Phases A + B: Boostback burn (convex guidance) and ballistic coast down
%% to landing-burn ignition altitude
bb = boostback_targeting(state0, p);   % convex boostback, aim biased for drag + entry burn
traj = simulate_boostback_coast(state0, bb, p);
tA = traj.tA; yA = traj.XA; kA = traj.kA; % kA: ZOH interval index of each sample
tB = traj.tB; yB = traj.XB;
stateB0 = yA(end,:)';

%% Phase C: Landing burn (SCvx guidance, re-planned in flight)
stateC0 = yB(end,:)';
landing = fly_landing_closed_loop(stateC0, p);
tC = landing.t; yC = landing.X;

%% Stitch phases together for reporting/plotting
t = [tA; tA(end)+tB; tA(end)+tB(end)+tC];
y = [yA; yB; yC];

% Thrust history, reconstructed the same way the EOM applied it
T_A = bb.aCmd(kA) .* yA(:,5);
T     = [T_A; traj.TB; landing.T];      % TB: entry burn (if enabled), else zeros
theta = [bb.theta(kA); traj.thetaB; landing.theta];

fprintf('--- MATLAB prototype: full mission ---\n');
fprintf('Boostback duration  : %.1f s, propellant %.0f kg (planned %.0f kg)\n', ...
        tA(end), state0(5) - stateB0(5), bb.propUsed);
fprintf('Coast duration      : %.1f s (planned T_coast %.1f s)\n', tB(end), bb.T_coast);
fprintf('Ignition-point miss : dx = %.2f m (target x = %.0f m)\n', ...
        stateC0(1) - bb.rTarget(1), bb.rTarget(1));
fprintf('Speed at ignition   : %.0f m/s\n', norm(stateC0(3:4)));
fprintf('Landing burn        : %.1f s, %d plans, %.0f kg\n', tC(end), landing.nSolves, landing.propUsed);
fprintf('Touchdown           : x = %+.2f m from the pad, v = (%+.2f, %+.2f) m/s\n', ...
        landing.touchdown(1) - p.target.x, landing.touchdown(3:4));
fprintf('Final state [x y vx vy m]:\n');
disp(y(end,:));

%% Save the flight as a record, then plot from the file
ev.tCutoff = tA(end);
ev.tIgnite = tA(end) + tB(end);
ev.tEntry  = tA(end) + traj.tEntry;   % [] if no entry burn
raw = struct('t', t, 'X', y, 'T', T, 'theta', theta, 'ev', ev);
rec = make_flight_record(raw, p, Name="prototype", Source="MATLAB prototype");
file = save_flight_record(rec);
fprintf('Saved flight record: %s\n', file);
plot_flight_record(file);

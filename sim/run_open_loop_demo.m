%% RUN_OPEN_LOOP_DEMO
% Phase 1 sanity-check script: propagates the planar 3DOF dynamics
% through a boostback burn, ballistic coast, and a landing burn, using
% PLACEHOLDER guidance profiles (see guidance/*_guidance.m). The goal
% here is to validate the equations of motion, mass depletion, and
% event-based phase transitions -- not to hit the landing target yet.
% Closing the loop with the real convex guidance is Phases 2-3.

clear; clc;
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

p = vehicleParams();

state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];

odeOpts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @touchdownEvent);

%% Phase A: Boostback burn
bb = boostback_guidance(state0, p);
thrustFcn_A = @(t, s) deal(bb.T_const, bb.theta_const);
[tA, yA] = ode45(@(t,s) threeDOF_EOM(t, s, thrustFcn_A, p), ...
                  [0, bb.duration], state0, odeOpts);

%% Phase B: Ballistic coast (zero thrust)
thrustFcn_B = @(t, s) deal(0, 0);
stateB0 = yA(end,:)';
[tB, yB] = ode45(@(t,s) threeDOF_EOM(t, s, thrustFcn_B, p), ...
                  [0, 300], stateB0, odeOpts);

%% Phase C: Landing burn
ld = pdg_landing_guidance(stateB0, p);
thrustFcn_C = @(t, s) deal(ld.T_const, ld.theta_const);
stateC0 = yB(end,:)';
[tC, yC] = ode45(@(t,s) threeDOF_EOM(t, s, thrustFcn_C, p), ...
                  [0, ld.duration], stateC0, odeOpts);

%% Stitch phases together for reporting/plotting
t = [tA; tA(end)+tB; tA(end)+tB(end)+tC];
y = [yA; yB; yC];

fprintf('--- Phase 1 open-loop demo (placeholder guidance) ---\n');
fprintf('Boostback duration  : %.1f s\n', tA(end));
fprintf('Coast duration      : %.1f s\n', tB(end));
fprintf('Landing burn span   : %.1f s (event-terminated at touchdown if reached)\n', tC(end));
fprintf('Final state [x y vx vy m]:\n');
disp(y(end,:));

%% Plots
figure('Name', 'Booster Return - Open-Loop Demo');

subplot(2,2,1);
plot(y(:,1)/1000, y(:,2)/1000, 'LineWidth', 1.5); grid on;
xlabel('Downrange [km]'); ylabel('Altitude [km]');
title('Trajectory');

subplot(2,2,2);
plot(t, y(:,2)/1000, 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Altitude [km]');
title('Altitude vs Time');

subplot(2,2,3);
plot(t, vecnorm(y(:,3:4),2,2), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Speed vs Time');

subplot(2,2,4);
plot(t, y(:,5), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Mass [kg]');
title('Mass vs Time');

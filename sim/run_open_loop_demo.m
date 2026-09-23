%% RUN_OPEN_LOOP_DEMO
% Plain-MATLAB run of the full return sequence: boostback burn,
% ballistic coast, landing burn.
%   - Boostback: min-fuel convex (SOCP) guidance, flown open-loop as
%     T = aCmd(t) * m(t) with the ZOH profile from boostback_guidance.m
%   - Coast: unpowered, until descending through landing-burn ignition
%     altitude
%   - Landing: still the PLACEHOLDER profile (Phase 3), so touchdown is
%     hard -- the check here is that the coast arrives at the ignition
%     point the boostback aimed for.

clear; clc;
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

p = vehicleParams();

state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];

odeOpts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @touchdownEvent);

%% Phase A: Boostback burn (convex guidance, flown one ZOH interval at a time)
bb = boostback_guidance(state0, p);
N = numel(bb.t);
dtNode = bb.duration / N;
tA = 0; yA = state0';
for k = 1:N
    thrustFcn_A = @(t, s) deal(bb.aCmd(k) * s(5), bb.theta(k));
    [tk, yk] = ode45(@(t,s) threeDOF_EOM(t, s, thrustFcn_A, p), ...
                     [(k-1)*dtNode, k*dtNode], yA(end,:)', odeOpts);
    tA = [tA; tk(2:end)];   %#ok<AGROW>
    yA = [yA; yk(2:end,:)]; %#ok<AGROW>
end

%% Phase B: Ballistic coast (zero thrust) down to landing-burn ignition altitude
thrustFcn_B = @(t, s) deal(0, 0);
coastOpts = odeset(odeOpts, 'Events', @(t, s) altitudeEvent(t, s, p.landingIgniteAltitude));
stateB0 = yA(end,:)';
[tB, yB] = ode45(@(t,s) threeDOF_EOM(t, s, thrustFcn_B, p), ...
                  [0, 1000], stateB0, coastOpts);

%% Phase C: Landing burn
stateC0 = yB(end,:)';
ld = pdg_landing_guidance(stateC0, p);
thrustFcn_C = @(t, s) deal(ld.T_const, ld.theta_const);
[tC, yC] = ode45(@(t,s) threeDOF_EOM(t, s, thrustFcn_C, p), ...
                  [0, ld.duration], stateC0, odeOpts);

%% Stitch phases together for reporting/plotting
t = [tA; tA(end)+tB; tA(end)+tB(end)+tC];
y = [yA; yB; yC];

fprintf('--- Open-loop demo (convex boostback, placeholder landing) ---\n');
fprintf('Boostback duration  : %.1f s, propellant %.0f kg (planned %.0f kg)\n', ...
        tA(end), state0(5) - stateB0(5), bb.propUsed);
fprintf('Coast duration      : %.1f s (planned T_coast %.1f s)\n', tB(end), bb.T_coast);
fprintf('Ignition-point miss : dx = %.2f m (target x = %.0f m)\n', ...
        stateC0(1) - bb.rTarget(1), bb.rTarget(1));
fprintf('Speed at ignition   : %.0f m/s\n', norm(stateC0(3:4)));
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

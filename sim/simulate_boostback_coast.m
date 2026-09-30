function traj = simulate_boostback_coast(state0, bb, p)
%SIMULATE_BOOSTBACK_COAST Fly a boostback profile through the nonlinear
%   EOM, then descend to landing-burn ignition altitude (or ground
%   contact): unpowered, or -- if p.entry.enabled -- with an entry burn.
%
%   The ZOH profile is flown one interval at a time as T = aCmd(k) * m, so
%   the integrator never steps across a thrust discontinuity.
%
%   Entry burn: when descending through p.entry.startAltitude, the burn is
%   sized from the plant state at that moment (entry_burn_sizing: the
%   guidance's own predictor, as a flight computer would run it from its
%   navigation state), then flown retrograde at full thrust until the
%   planned cutoff mass.
%
%   traj fields:
%     tA, XA, kA - boostback time [s], states [x y vx vy m], and the ZOH
%                  interval index of each sample (for rebuilding thrust)
%     tB, XB     - post-cutoff time (from cutoff) and states: coast, entry
%                  burn if any, coast
%     TB, thetaB - thrust magnitude [N] and angle [rad] over tB
%     ignition   - state at the end of the descent
%     entry      - entry_burn_sizing result ([] if no entry burn)
%     tEntry     - [start end] of the entry burn in tB time ([] if none)

odeOpts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @touchdownEvent);

N = numel(bb.t);
dtNode = bb.duration / N;
tA = 0; XA = state0(:)'; kA = 1;
for k = 1:N
    thrustFcn = @(t, s) deal(bb.aCmd(k) * s(5), bb.theta(k));
    [tk, xk] = ode45(@(t, s) threeDOF_EOM(t, s, thrustFcn, p), ...
                     [(k-1)*dtNode, k*dtNode], XA(end,:)', odeOpts);
    tA = [tA; tk(2:end)];                    %#ok<AGROW>
    XA = [XA; xk(2:end,:)];                  %#ok<AGROW>
    kA = [kA; repmat(k, numel(tk) - 1, 1)];  %#ok<AGROW>
end

% Ballistic motion is a parabola, which ode45 integrates exactly, so its
% error control lets steps grow to tens of seconds. The accuracy is fine
% (~1e-11 m), but plots would join a handful of points with straight
% lines. Ask for output every dtOut seconds instead (ode45 interpolates
% between its own steps; the terminal event is still located exactly).
dtOut = 0.5;
coast = @(t0, x0, hEvent) ode45(@(t, s) threeDOF_EOM(t, s, @(~, ~) deal(0, 0), p), ...
    t0:dtOut:(t0 + 5 * bb.T_coast + 1000), x0, ...
    odeset(odeOpts, 'Events', @(t, s) altitudeEvent(t, s, hEvent)));

entry = [];
tEntry = [];
if isfield(p, 'entry') && p.entry.enabled
    [tB, XB] = coast(0, XA(end,:)', p.entry.startAltitude);
    TB = zeros(size(tB)); thetaB = zeros(size(tB));
    reachedStart = abs(XB(end,2) - p.entry.startAltitude) < 1 && XB(end,4) < 0;
    if reachedStart
        entry = entry_burn_sizing(XB(end,:)', p);
        if entry.dv > 0
            retro = @(t, s) deal(p.T_max_entry, atan2(-s(4), -s(3)));
            [tE, XE] = ode45(@(t, s) threeDOF_EOM(t, s, retro, p), ...
                tB(end):dtOut:(tB(end) + 600), XB(end,:)', ...
                odeset(odeOpts, 'Events', @(t, s) massEvent(t, s, entry.mCutoff)));
            tEntry = [tE(1) tE(end)];
            tB = [tB; tE(2:end)];
            XB = [XB; XE(2:end,:)];
            TB = [TB; p.T_max_entry * ones(numel(tE) - 1, 1)];
            thetaB = [thetaB; atan2(-XE(2:end,4), -XE(2:end,3))];
        end
        [tC, XC] = coast(tB(end), XB(end,:)', p.landingIgniteAltitude);
        tB = [tB; tC(2:end)];
        XB = [XB; XC(2:end,:)];
        TB = [TB; zeros(numel(tC) - 1, 1)];
        thetaB = [thetaB; zeros(numel(tC) - 1, 1)];
    else
        % Apogee below the entry start altitude: no entry burn, plain coast
        [tB, XB] = coast(0, XA(end,:)', p.landingIgniteAltitude);
        TB = zeros(size(tB)); thetaB = zeros(size(tB));
    end
else
    [tB, XB] = coast(0, XA(end,:)', p.landingIgniteAltitude);
    TB = zeros(size(tB)); thetaB = zeros(size(tB));
end

traj = struct('tA', tA, 'XA', XA, 'kA', kA, 'tB', tB, 'XB', XB, ...
              'TB', TB, 'thetaB', thetaB, 'ignition', XB(end,:)', ...
              'entry', entry, 'tEntry', tEntry);
end

function [value, isterminal, direction] = massEvent(~, s, mCut)
% Engine cutoff when the planned propellant has been burned.
value = s(5) - mCut; isterminal = 1; direction = -1;
end

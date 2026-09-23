function traj = simulate_boostback_coast(state0, bb, p)
%SIMULATE_BOOSTBACK_COAST Fly a boostback profile through the nonlinear
%   EOM, then coast unpowered until descending through landing-burn
%   ignition altitude (or ground contact).
%
%   The ZOH profile is flown one interval at a time as T = aCmd(k) * m, so
%   the integrator never steps across a thrust discontinuity.
%
%   traj fields:
%     tA, XA, kA - boostback time [s], states [x y vx vy m], and the ZOH
%                  interval index of each sample (for rebuilding thrust)
%     tB, XB     - coast time (from cutoff) and states
%     ignition   - state at the end of the coast

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

coastOpts = odeset(odeOpts, 'Events', @(t, s) altitudeEvent(t, s, p.landingIgniteAltitude));
[tB, XB] = ode45(@(t, s) threeDOF_EOM(t, s, @(~, ~) deal(0, 0), p), ...
                 [0, 5 * bb.T_coast + 100], XA(end,:)', coastOpts);

traj = struct('tA', tA, 'XA', XA, 'kA', kA, 'tB', tB, 'XB', XB, ...
              'ignition', XB(end,:)');
end

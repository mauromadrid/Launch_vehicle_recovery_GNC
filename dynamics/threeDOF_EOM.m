function dstate = threeDOF_EOM(t, state, thrustFcn, p)
%THREEDOF_EOM Planar (downrange/altitude) point-mass equations of motion
%   for a thrust-vectored, mass-varying vehicle. No aerodynamic forces
%   are modeled yet (Phase 1 scope) — exo-/thin-atmosphere boostback,
%   coast, and landing-burn phases only.
%
%   state = [x; y; vx; vy; m]
%
%   thrustFcn(t, state) must return [T, theta]:
%       T     - thrust magnitude [N] (0 <= T <= p.T_max)
%       theta - thrust direction angle [rad], measured from the
%               positive x-axis (0 = horizontal downrange, pi/2 = straight up)
%
%   p - parameter struct from vehicleParams()

vx = state(3);
vy = state(4);
m  = state(5);

[T, theta] = thrustFcn(t, state);

% Guard against non-physical mass depletion during integration.
if m <= p.dryMass
    T = 0;
end

ax = (T / m) * cos(theta);
ay = (T / m) * sin(theta) - p.g;
mdot = -T / (p.Isp * p.g0);

dstate = [vx; vy; ax; ay; mdot];
end

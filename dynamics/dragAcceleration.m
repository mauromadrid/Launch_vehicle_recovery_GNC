function [aDrag, info] = dragAcceleration(state, p)
%DRAGACCELERATION Aerodynamic drag acceleration on the stage (plant model).
%
%   [aDrag, info] = dragAcceleration(state, p)
%     state - [x; y; vx; vy; m]
%     aDrag - [ax; ay] drag acceleration [m/s^2], opposite to the velocity
%     info  - struct: rho [kg/m^3], mach, CD, q (dynamic pressure) [Pa],
%             decel (|aDrag|) [m/s^2]
%
%   D = 0.5*rho*V^2*CD(Mach)*A_ref along -v_hat, so
%   aDrag = -(0.5*rho*V*CD*A_ref/m) * v, which avoids dividing by V (no
%   singularity at V = 0). No wind yet: air-relative = inertial velocity.
%
%   Atmosphere: COESA 1976 (atmoscoesa), altitude clamped to
%   [0, p.atmo.maxAltitude] -- see vehicleParams for why.

v = state(3:4);
V = norm(v);
m = state(5);

info = struct('rho', 0, 'mach', 0, 'CD', 0, 'q', 0, 'decel', 0);
if ~p.aero.dragEnabled
    aDrag = [0; 0];
    return
end

h = min(max(state(2), 0), p.atmo.maxAltitude);
[~, a, ~, rho] = atmoscoesa(h, 'None');

mach = V / a;
CD = interp1(p.aero.mach, p.aero.CD, min(mach, p.aero.mach(end)), 'linear');

k = 0.5 * rho * V * CD * p.aero.A_ref / m;
aDrag = -k * v;

info = struct('rho', rho, 'mach', mach, 'CD', CD, 'q', 0.5 * rho * V^2, ...
              'decel', k * V);
end

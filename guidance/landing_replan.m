function [Xs, Us, s, m0, tNodes, aCmd, theta, duration, ok] = landing_replan( ...
        state, p, hasPrev, prevX, prevU, prevS, prevM0, elapsed)
%LANDING_REPLAN One landing-guidance solve, reduced to plain numeric arrays
%   for the Simulink model's MATLAB Function block (which calls it
%   extrinsically and keeps the arrays as its state).
%
%   [Xs, Us, s, m0, tNodes, aCmd, theta, duration, ok] = landing_replan( ...
%       state, p, hasPrev, prevX, prevU, prevS, prevM0, elapsed)
%
%   state   - measured [x; y; vx; vy; m]
%   hasPrev - false: cold start (first plan, at ignition); true: warm start
%             from the previous plan (prevX, prevU, prevS, prevM0 = its
%             scaled solution, elapsed = seconds since it was made)
%   Outputs - the new plan's scaled solution (for the next warm start), its
%             node times / thrust-acceleration magnitudes / thrust angles
%             (N-1 x 1 each), its duration, and ok = converged.
%   On failure the outputs are empty-safe zeros with ok = false, and the
%   caller keeps flying its previous plan.

ws = [];
if hasPrev
    ws = struct('X', prevX, 'U', prevU, 's', prevS, 'm0', prevM0, 'elapsed', elapsed);
end
try
    ld = landing_solve(state, p, ws);        % thrust margin + full-thrust fallback
    ok = ld.converged;
catch
    ok = false;
end
N = p.landing.N;
if ok
    Xs = ld.Xs; Us = ld.Us; s = ld.s; m0 = ld.m0;
    tNodes = ld.t; aCmd = ld.aCmd; theta = ld.theta; duration = ld.duration;
else
    Xs = zeros(5, N); Us = zeros(3, N-1); s = 0; m0 = 0;
    tNodes = (0:N-2)'; aCmd = zeros(N-1, 1); theta = zeros(N-1, 1); duration = 0;
end
end

function thrustProfile = boostback_guidance(state0, p) %#ok<INUSD>
%BOOSTBACK_GUIDANCE Placeholder for the boostback-burn guidance solve.
%   Phase 2 of the project: formulate and solve a convex (SOCP)
%   minimum-fuel burn that reverses enough downrange velocity to put
%   the subsequent ballistic coast on a path back toward the landing
%   target, using MATLAB's coneprog (Optimization Toolbox).
%
%   For now this returns a fixed, hand-picked burn (constant thrust
%   and angle for a fixed duration) purely so the full open-loop
%   pipeline (boostback -> coast -> landing) can be exercised and the
%   dynamics validated before the guidance solve is implemented.
%
%   TODO (Phase 2):
%     1. Define the discretized dynamics (zero-order hold over N nodes)
%     2. Apply lossless convexification to the thrust lower bound
%     3. Pose as an SOCP: minimize fuel s.t. dynamics, thrust bounds,
%        pointing constraint, boundary conditions
%     4. Solve with coneprog and return the resulting profile

warning('boostback_guidance:placeholder', ...
    'Using placeholder fixed-burn boostback profile -- convex solve not yet implemented.');

thrustProfile.T_const     = 0.9 * p.T_max;
thrustProfile.theta_const = deg2rad(200); % roughly retrograde, arbitrary for now
thrustProfile.duration    = 12; % [s]
end

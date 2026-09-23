function [T_coast, vNeeded] = estimate_coast_duration(r0, v0, rTarget, p)
%ESTIMATE_COAST_DURATION Pick the fixed ballistic-coast duration used by
%   the boostback guidance's affine terminal constraint.
%
%   For a candidate coast duration T, the velocity that would carry the
%   vehicle from r0 to rTarget in exactly T seconds of vacuum,
%   constant-g ballistic flight is
%
%       vx_needed(T) = (x_target - x0) / T
%       vy_needed(T) = (y_target - y0 + g*T^2/2) / T
%
%   T_coast is the T that minimizes ||v_needed(T) - v0||^2, i.e. the
%   coast arc that needs the smallest impulsive velocity change from
%   the current velocity. This is a cheap 1D scalar minimization
%   (fminbnd), so it can be redone every guidance cycle if replanning
%   is added later.
%
%   Known first-order approximations (see CLAUDE.md): no drag, and r0
%   is the current (separation) position rather than the not-yet-known
%   burn-cutoff position. Neither affects targeting accuracy -- the
%   boostback SOCP enforces the affine constraint on the true
%   burn-cutoff state -- only how close to fuel-optimal T_coast is.
%
%   Inputs:
%     r0, v0   - current position / velocity [x; y] [m], [m/s]
%     rTarget  - coast end point [x; y] [m] (landing-burn ignition point)
%     p        - parameter struct from vehicleParams()
%
%   Outputs:
%     T_coast  - chosen coast duration [s]
%     vNeeded  - [vx; vy] needed at the start of the coast [m/s]

g = p.g;
dr = rTarget(:) - r0(:);

vNeededFcn = @(T) [dr(1) ./ T; (dr(2) + 0.5 * g * T.^2) ./ T];
J = @(T) sum((vNeededFcn(T) - v0(:)).^2);

% Upper bound: generous multiple of the free-fall time from r0, so the
% search interval always contains the minimizer for sane inputs.
T_upper = 4 * (abs(v0(2)) + sqrt(v0(2)^2 + 2 * g * max(-dr(2), 0))) / g + 100;
T_coast = fminbnd(J, 1, T_upper, optimset('TolX', 1e-6));
vNeeded = vNeededFcn(T_coast);
end

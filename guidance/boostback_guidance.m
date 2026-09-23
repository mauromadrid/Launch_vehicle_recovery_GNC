function bb = boostback_guidance(state0, p)
%BOOSTBACK_GUIDANCE Minimum-fuel boostback burn via lossless-convexified
%   SOCP, solved with coneprog (Optimization Toolbox).
%
%   The burn is not aimed at the landing site itself: it sets up an
%   unpowered ballistic coast of fixed duration T_coast that ends at the
%   landing-burn ignition point [p.target.x; p.landingIgniteAltitude].
%   Because vacuum, constant-g coast dynamics are linear, "coasting
%   T_coast seconds from the burn-cutoff state reaches the target" is an
%   affine equality on the cutoff state -- which keeps the problem convex.
%   T_coast itself is fixed beforehand by estimate_coast_duration().
%
%   Formulation (Acikmese & Ploen 2007 change of variables):
%       u = T/m (thrust acceleration), sigma = Gamma/m, z = ln(m/m0)
%   which makes the dynamics linear:
%       r' = v,  v' = u + g,  z' = -sigma/(Isp*g0)
%   and the problem, over N zero-order-hold intervals of a burn of
%   duration t_b, is
%       minimize    sum_k sigma_k * dt           (= delta-v = fuel)
%       subject to  exact ZOH-discretized dynamics above
%                   ||u_k|| <= sigma_k                        (SOC)
%                   n'*u_k >= cos(maxTiltAngle) * sigma_k     (pointing)
%                   T_min*e^-z <= sigma <= T_max*e^-z  (Taylor-expanded
%                       about the max-thrust mass profile: SOC lower,
%                       linear upper bound)
%                   r_0, v_0, z_0 = current state
%                   x_N + vx_N*T_coast              = x_target
%                   y_N + vy_N*T_coast - g*T_coast^2/2 = y_target
%                   m_N >= dryMass + landingPropReserve
%
%   The pointing cone is about a reference axis n, the direction of the
%   impulsive delta-v needed to enter the coast arc (the vehicle flips
%   to a boostback attitude and may only deviate maxTiltAngle from it).
%   Phase 3's landing burn will use the same constraint with n = up.
%
%   Free burn duration is non-convex, so t_b is handled by an outer 1D
%   search (coarse grid + fminbnd), each evaluation being one SOCP --
%   the standard G-FOLD treatment of free final time.
%
%   bb = boostback_guidance(state0, p)
%     state0 - [x; y; vx; vy; m] at burn start; [] = separation state
%     p      - parameter struct from vehicleParams()
%
%   bb fields (profile is piecewise constant on [t(k), t(k)+dt)):
%     t          - N x 1 node start times, relative to burn start [s]
%     aCmd       - N x 1 thrust-acceleration magnitude command [m/s^2]
%     theta      - N x 1 thrust angle from +x axis [rad]
%     T_nominal  - N x 1 thrust magnitude on the planned mass profile [N]
%     duration   - burn duration t_b [s]
%     T_coast    - fixed coast duration used in the terminal constraint [s]
%     rTarget    - coast end point the burn is aimed at [m]
%     r, v, m    - planned node states (N+1 columns / entries)
%     cutoffState- planned [x; y; vx; vy; m] at burn cutoff
%     propUsed   - planned boostback propellant [kg]
%     relaxationGap - max_k (sigma_k - ||u_k||)/sigma_k; ~0 confirms the
%                     convexification was lossless
%
%   The profile is meant to be flown as T = aCmd(t) * m(t) with the
%   vehicle's actual mass, which reproduces the planned acceleration
%   exactly (and hence the planned trajectory, up to integration error).

if nargin < 1 || isempty(state0)
    state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];
end
r0 = state0(1:2);
v0 = state0(3:4);
m0 = state0(5);

rTarget = [p.target.x; p.landingIgniteAltitude];
[T_coast, vNeeded] = estimate_coast_duration(r0, v0, rTarget, p);

dvDir = vNeeded - v0;
nRef = dvDir / norm(dvDir);

mFinalMin = p.dryMass + p.landingPropReserve;
if m0 <= mFinalMin
    error('boostback_guidance:noPropellant', ...
        'Initial mass %.0f kg leaves no propellant above the %.0f kg reserve floor.', ...
        m0, mFinalMin);
end

prob = struct('r0', r0, 'v0', v0, 'm0', m0, 'rTarget', rTarget, ...
              'T_coast', T_coast, 'nRef', nRef, 'mFinalMin', mFinalMin, ...
              'N', p.boostbackNodes);

% ---- Outer search over burn duration ----
% Engine can't throttle below T_min, so the longest possible burn spends
% the whole propellant budget at T_min.
tbMax = (m0 - mFinalMin) * p.Isp * p.g0 / p.T_min;
tbGrid = linspace(0.05 * tbMax, tbMax, 20);
fuelGrid = arrayfun(@(tb) fuelForBurnDuration(tb, prob, p), tbGrid);

[fuelBest, iBest] = min(fuelGrid);
if ~isfinite(fuelBest)
    error('boostback_guidance:infeasible', ...
        ['No feasible boostback burn found for burn durations up to %.1f s. ', ...
         'Required impulsive delta-v is %.0f m/s; available (to the reserve ', ...
         'floor) is %.0f m/s. Check the separation state / propellant in vehicleParams.'], ...
        tbMax, norm(dvDir), p.Isp * p.g0 * log(m0 / mFinalMin));
end
tbLo = tbGrid(max(iBest - 1, 1));
tbHi = tbGrid(min(iBest + 1, numel(tbGrid)));
tbOpt = fminbnd(@(tb) fuelForBurnDuration(tb, prob, p), tbLo, tbHi, ...
                optimset('TolX', 1e-3));
if fuelForBurnDuration(tbOpt, prob, p) > fuelBest
    tbOpt = tbGrid(iBest); % fminbnd can land on an infeasible edge
end

sol = solveBoostbackSOCP(tbOpt, prob, p);

% ---- Package the result ----
N = prob.N;
dt = tbOpt / N;
bb.t         = (0:N-1)' * dt;
bb.aCmd      = sol.sigma(:);
bb.theta     = atan2(sol.u(2,:), sol.u(1,:))';
bb.T_nominal = sol.sigma(:) .* sol.m(1:N)';
bb.duration  = tbOpt;
bb.T_coast   = T_coast;
bb.rTarget   = rTarget;
bb.nRef      = nRef;
bb.r         = sol.r;
bb.v         = sol.v;
bb.m         = sol.m;
bb.cutoffState = [sol.r(:,end); sol.v(:,end); sol.m(end)];
bb.propUsed  = m0 - sol.m(end);
bb.relaxationGap = max((sol.sigma - vecnorm(sol.u)) ./ sol.sigma);
end

% =====================================================================
function fuel = fuelForBurnDuration(tb, prob, p)
% Propellant used by the optimal burn of duration tb (Inf if infeasible).
sol = solveBoostbackSOCP(tb, prob, p);
if sol.feasible
    fuel = prob.m0 - sol.m(end);
else
    fuel = Inf;
end
end

% =====================================================================
function sol = solveBoostbackSOCP(tb, prob, p)
% Build and solve the fixed-burn-duration SOCP in nondimensional units
% (length 1 km, acceleration 1 g => time sqrt(1000/g) s), which keeps all
% decision variables O(1)-O(100) for the interior-point solver.

N = prob.N;
Lsc = 1e3;
Tsc = sqrt(Lsc / p.g);
Vsc = Lsc / Tsc;
Asc = p.g;

dt    = tb / N / Tsc;
Tcst  = prob.T_coast / Tsc;
gVec  = [0; -1];
alpha = Asc * Tsc / (p.Isp * p.g0);          % z' = -alpha * sigma
rho1  = p.T_min / (prob.m0 * Asc);           % thrust bounds as accel at m0
rho2  = p.T_max / (prob.m0 * Asc);

% Mass-profile bounds: fastest (T_max) and slowest (T_min) possible burn.
% zLo is also the Taylor-expansion point for the thrust bounds.
tNodes = (0:N)' * dt;
zLo = log(max(1 - alpha * rho2 * tNodes, p.dryMass / prob.m0));
zHi = log(1 - alpha * rho1 * tNodes);

% ---- Decision-vector layout: [r(2,N+1) v(2,N+1) z(N+1) u(2,N) sigma(N)]
nX = N + 1;
iR = reshape(1:2*nX, 2, nX);
iV = 2*nX + reshape(1:2*nX, 2, nX);
iZ = 4*nX + (1:nX);
iU = 5*nX + reshape(1:2*N, 2, N);
iS = 5*nX + 2*N + (1:N);
nVar = 5*nX + 3*N;

% ---- Equality constraints ----
nEq = 5*N + 5 + 2;
Aeq = spalloc(nEq, nVar, 20*N + 20);
beq = zeros(nEq, 1);
row = 0;
for k = 1:N
    for ax = 1:2
        % r_{k+1} = r_k + dt*v_k + dt^2/2*(u_k + g)
        row = row + 1;
        Aeq(row, [iR(ax,k+1) iR(ax,k) iV(ax,k) iU(ax,k)]) = [1 -1 -dt -dt^2/2]; %#ok<SPRIX>
        beq(row) = dt^2/2 * gVec(ax);
        % v_{k+1} = v_k + dt*(u_k + g)
        row = row + 1;
        Aeq(row, [iV(ax,k+1) iV(ax,k) iU(ax,k)]) = [1 -1 -dt]; %#ok<SPRIX>
        beq(row) = dt * gVec(ax);
    end
    % z_{k+1} = z_k - alpha*dt*sigma_k
    row = row + 1;
    Aeq(row, [iZ(k+1) iZ(k) iS(k)]) = [1 -1 alpha*dt]; %#ok<SPRIX>
end
% Initial state
x0 = [prob.r0 / Lsc; prob.v0 / Vsc; 0];
Aeq(row+1:row+5, [iR(:,1); iV(:,1); iZ(1)]) = eye(5);
beq(row+1:row+5) = x0;
row = row + 5;
% Terminal affine coast constraint
rT = prob.rTarget / Lsc;
row = row + 1;
Aeq(row, [iR(1,end) iV(1,end)]) = [1 Tcst];
beq(row) = rT(1);
row = row + 1;
Aeq(row, [iR(2,end) iV(2,end)]) = [1 Tcst];
beq(row) = rT(2) + 0.5 * Tcst^2;

% ---- Linear inequality constraints ----
cosTilt = cos(p.maxTiltAngle);
mu1 = rho1 * exp(-zLo(1:N));
mu2 = rho2 * exp(-zLo(1:N));
A = spalloc(2*N + 1, nVar, 6*N + 1);
b = zeros(2*N + 1, 1);
for k = 1:N
    % Pointing cone: n'*u_k >= cos(tilt)*sigma_k
    A(k, [iU(:,k); iS(k)]) = [-prob.nRef' cosTilt]; %#ok<SPRIX>
    % Upper thrust bound, 1st-order Taylor of T_max*e^-z (conservative)
    A(N+k, [iS(k) iZ(k)]) = [1 mu2(k)]; %#ok<SPRIX>
    b(N+k) = mu2(k) * (1 + zLo(k));
end
% Landing propellant reserve
A(2*N+1, iZ(end)) = -1;
b(2*N+1) = -log(prob.mFinalMin / prob.m0);

% ---- Second-order cone constraints ----
cones = repmat(secondordercone(sparse(1, nVar), 0, sparse(nVar, 1), 0), 1, 2*N);
for k = 1:N
    % Lossless-convexification slack: ||u_k|| <= sigma_k
    Ak = sparse([1 2], iU(:,k), [1 1], 2, nVar);
    dk = sparse(iS(k), 1, 1, nVar, 1);
    cones(k) = secondordercone(Ak, [0; 0], dk, 0);

    % Lower thrust bound, 2nd-order Taylor of T_min*e^-z:
    %   mu1*(1 - dz + dz^2/2) <= sigma,  dz = z_k - zLo_k
    % i.e.  (mu1/2)*dz^2 <= L,  L = sigma + mu1*z - mu1*(1 + zLo)
    % as the cone  ||[sqrt(2*mu1)*dz; L - 1]|| <= L + 1
    c0 = -mu1(k) * (1 + zLo(k));
    aL = sparse([iS(k) iZ(k)], 1, [1 mu1(k)], nVar, 1);
    Ak = [sparse(1, iZ(k), sqrt(2*mu1(k)), 1, nVar); aL'];
    bk = [sqrt(2*mu1(k)) * zLo(k); 1 - c0];
    cones(N+k) = secondordercone(Ak, bk, aL, -(c0 + 1));
end

% ---- Bounds ----
lb = -inf(nVar, 1);
ub =  inf(nVar, 1);
lb(iZ) = zLo;
ub(iZ) = zHi;
lb(iS) = 0;

% ---- Cost: total delta-v, sum(sigma_k)*dt ----
f = zeros(nVar, 1);
f(iS) = dt;

opts = optimoptions('coneprog', 'Display', 'off');
[xOpt, ~, exitflag] = coneprog(f, cones, A, b, Aeq, beq, lb, ub, opts);

sol.feasible = exitflag == 1;
sol.exitflag = exitflag;
if ~sol.feasible
    return
end
sol.r     = xOpt(iR) * Lsc;
sol.v     = xOpt(iV) * Vsc;
sol.m     = prob.m0 * exp(xOpt(iZ)');
sol.u     = xOpt(iU) * Asc;
sol.sigma = xOpt(iS)' * Asc;
end

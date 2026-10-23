function ld = landing_guidance_scvx(state0, p, opts)
%LANDING_GUIDANCE_SCVX Minimum-fuel landing burn with drag, by successive
%   convexification (SCvx: Mao, Szmuk & Acikmese), solved with coneprog.
%
%   ld = landing_guidance_scvx(state0, p)
%   ld = landing_guidance_scvx(state0, p, N=31, MaxIter=30, Verbose=true)
%
%   state0 - [x; y; vx; vy; m] at landing-burn ignition
%
%   Problem (normalized time tau in [0,1], t = tau*s, s = burn duration):
%       maximize   final mass                        (= minimum fuel)
%       subject to dx/dtau = s * F(x, u)             nonlinear: drag (guidance
%                                                    model: exponential
%                                                    atmosphere, CD(Mach))
%                  ||u|| <= sigma                    lossless-convexification slack
%                  T_min e^-z <= sigma <= T_max e^-z (Taylor about the reference)
%                  u_y >= cos(maxTiltAngle) sigma    pointing, about vertical
%                  |x - x_pad| <= y cot(glideSlope)  glide slope (2 linear rows)
%                  x(0) = state0;  r(1) = pad, v(1) = 0;  m >= dry mass
%   with x = [x; y; vx; vy; z = ln(m/m0)], u = [ux; uy; sigma] (thrust
%   acceleration and slack), zero-order hold on u over N-1 intervals.
%
%   SCvx loop (each iteration one SOCP):
%     1. linearize the dynamics about the reference (xbar, ubar, sbar) and
%        discretize exactly along it:  x_{k+1} = A_k x_k + B_k u_k + S_k s
%        + c_k (+ nu_k), where c_k makes the model exact at the reference
%     2. virtual control nu_k, penalized lambda*||nu||_1: keeps every
%        subproblem feasible while the reference is still poor
%     3. trust region |x - xbar|, |u - ubar|, |s - sbar| <= r (scaled): only
%        trust the linearization near the reference
%     4. accept/reject by rho = actual / predicted reduction of the
%        nonlinear penalized cost J = -z_N + lambda*sum|defects|, and
%        grow/shrink r accordingly
%   The burn duration s is a decision variable (time dilation), so no outer
%   search over burn time is needed.
%
%   ld fields (profile piecewise constant on [t(k), t(k+1))):
%     t, aCmd, theta - node start times [s], thrust-acceleration magnitude
%                      [m/s^2] (= ||u||, not sigma), thrust angle [rad]
%     duration       - burn duration [s]
%     r, v, m        - planned node states
%     propUsed, converged, iterations, maxDefect, relaxationGap
%     history        - per-iteration table (cost, defect, trust radius, rho)

arguments
    state0 (5,1) double
    p struct
    opts.N (1,1) double = 31
    opts.MaxIter (1,1) double = 30
    opts.Verbose (1,1) logical = false
    opts.Lambda (1,1) double = 1e3
    opts.SolverDisplay (1,1) string = "off"
end

N = opts.N;
K = N - 1;                         % intervals
dtau = 1 / K;

% ---- Scaling: length 1 km, acceleration 1 g ----
sc.L = 1e3; sc.A = p.g; sc.T = sqrt(sc.L / sc.A); sc.V = sc.L / sc.T;
m0 = state0(5);
sc.m0 = m0;
alpha = sc.A * sc.T / (p.Isp * p.g0);           % dz/dt~ = -alpha*sigma~
rho1 = p.T_min / (m0 * sc.A);                   % thrust bounds as accel at m0
rho2 = p.T_max / (m0 * sc.A);
zDry = log(p.dryMass / m0);
cosTilt = cos(p.maxTiltAngle);
cotGS = cot(p.glideSlopeAngle);

x0 = [state0(1:2) / sc.L; state0(3:4) / sc.V; 0];
xPad = p.target.x / sc.L;
model = @(x, u) dynamicsF(x, u, p, sc, alpha);

% ---- Initial guess ----
% Kinematically consistent cubic (Hermite) path matching position and
% velocity at both ends, over a constant-deceleration time estimate. The
% controls that path needs are projected into the convex constraint set
% (tilt cone, thrust bounds), and mass is integrated with them. A
% reference violating hard constraints would make the first trust-region
% subproblems infeasible.
V0 = norm(state0(3:4));
tf0 = 2 * hypot(state0(2), state0(1) - p.target.x) / max(V0, 1);
sbar = tf0 / sc.T;
tau = linspace(0, 1, N);
r0 = x0(1:2); v0 = x0(3:4); r1 = [xPad; 0];
h00 = 2*tau.^3 - 3*tau.^2 + 1;  h10 = tau.^3 - 2*tau.^2 + tau;  h01 = -2*tau.^3 + 3*tau.^2;
d00 = 6*tau.^2 - 6*tau;         d10 = 3*tau.^2 - 4*tau + 1;     d01 = -6*tau.^2 + 6*tau;
xbar = zeros(5, N);
xbar(1:2,:) = r0 .* h00 + sbar * v0 .* h10 + r1 .* h01;     % d/dtau = sbar * d/dt (scaled)
xbar(3:4,:) = (r0 .* d00 + sbar * v0 .* d10 + r1 .* d01) / sbar;
ubar = zeros(3, K);
for k = 1:K
    aNeeded = (xbar(3:4,k+1) - xbar(3:4,k)) / (dtau * sbar);   % scaled accel
    Fk = model(xbar(:,k), [0; 0; 0]);                          % gravity + drag
    ubar(:,k) = projectControl(aNeeded - Fk(3:4), xbar(5,k), rho1, rho2, p.maxTiltAngle);
    xbar(5,k+1) = max(xbar(5,k) - alpha * ubar(3,k) * dtau * sbar, zDry);
end

% ---- SCvx parameters ----
lambda = opts.Lambda;
r = 1.0; rMin = 1e-5; rMax = 5;
rho0 = 0.0; rho1Acc = 0.25; rho2Acc = 0.7;
Jbar = nonlinearCost(xbar, ubar, sbar, model, dtau, lambda);
hist = zeros(0, 6);
converged = false;
coneOpts = optimoptions('coneprog', 'Display', opts.SolverDisplay);

for it = 1:opts.MaxIter
    [Ak, Bk, Sk, ck] = discretize(xbar, ubar, sbar, model, dtau);
    [sol, ok] = solveSubproblem(Ak, Bk, Sk, ck, xbar, ubar, sbar, r);
    if ~ok
        if opts.Verbose
            fprintf('  it %2d  subproblem not solved (coneprog exitflag %d), r %.3g -> %.3g\n', ...
                    it, sol.flag, r, r/2);
        end
        r = r / 2;                                   % infeasible/numerical: shrink, retry
        hist(end+1,:) = [it, Jbar, NaN, NaN, r, NaN]; %#ok<AGROW>
        if r < rMin, break, end
        continue
    end
    Jnew = nonlinearCost(sol.X, sol.U, sol.s, model, dtau, lambda);
    predicted = Jbar - sol.L;            % improvement the linear model promises
    actual = Jbar - Jnew;                % improvement the real dynamics deliver
    maxDef = maxDefect(sol.X, sol.U, sol.s, model, dtau);
    tolJ = 1e-4 * max(1, abs(Jbar));     % below this, cost changes are numerical noise

    % Converged: the linear model sees nothing left to gain and the real
    % dynamics are satisfied (no virtual control needed).
    if predicted < tolJ && maxDef < 1e-6
        if Jnew <= Jbar + tolJ
            xbar = sol.X; ubar = sol.U; sbar = sol.s; Jbar = Jnew;
        end
        hist(end+1,:) = [it, Jbar, maxDef, max(abs(sol.nu(:))), r, NaN]; %#ok<AGROW>
        if opts.Verbose
            fprintf('  it %2d  J %+.6f  defect %.2e  converged\n', it, Jbar, maxDef);
        end
        converged = true;
        break
    end

    % Accept only steps that are predicted to improve AND actually do
    % (with predicted <= 0 the ratio's sign would be meaningless).
    ratio = actual / max(predicted, eps);
    accepted = predicted > 0 && ratio > rho0;
    hist(end+1,:) = [it, Jnew, maxDef, max(abs(sol.nu(:))), r, ratio]; %#ok<AGROW>
    if opts.Verbose
        fprintf('  it %2d  J %+.6f  defect %.2e  nu %.2e  r %.3g  rho %.2f%s\n', it, Jnew, ...
                maxDef, max(abs(sol.nu(:))), r, ratio, string(repmat(' (rejected)', 1, ~accepted)));
    end
    if accepted
        xbar = sol.X; ubar = sol.U; sbar = sol.s; Jbar = Jnew;
    end
    if ratio < rho1Acc
        r = max(r / 2, rMin);
    elseif ratio > rho2Acc
        r = min(r * 2, rMax);
    end
end

% ---- Package (physical units) ----
ld.duration  = sbar * sc.T;
ld.t         = (0:K-1)' * dtau * ld.duration;
ld.aCmd      = vecnorm(ubar(1:2,:))' * sc.A;
ld.theta     = atan2(ubar(2,:), ubar(1,:))';
ld.sigma     = ubar(3,:)' * sc.A;
ld.r         = xbar(1:2,:) * sc.L;
ld.v         = xbar(3:4,:) * sc.V;
ld.m         = m0 * exp(xbar(5,:));
ld.propUsed  = m0 - ld.m(end);
ld.converged = converged;
ld.iterations = size(hist, 1);
ld.maxDefect = maxDefect(xbar, ubar, sbar, model, dtau);
ld.relaxationGap = max((ubar(3,:) - vecnorm(ubar(1:2,:))) ./ ubar(3,:));
ld.history = array2table(hist, 'VariableNames', ...
    {'iter', 'J', 'maxDefect', 'maxVirtualControl', 'trustRadius', 'rho'});

% =====================================================================
    function [sol, ok] = solveSubproblem(Ak, Bk, Sk, ck, xb, uRef, sb, rTR)
        nX = 5*N; nU = 3*K;
        iX = reshape(1:nX, 5, N);
        iU = nX + reshape(1:nU, 3, K);
        iS = nX + nU + 1;
        iNu = iS + reshape(1:5*K, 5, K);
        iEta = iNu(end) + reshape(1:5*K, 5, K);
        nVar = iEta(end);

        % Equalities: linearized dynamics + virtual control, boundary conditions
        nEq = 5*K + 5 + 4;
        Aeq = spalloc(nEq, nVar, 5*K*14 + 9);
        beq = zeros(nEq, 1);
        for kk = 1:K
            rows = 5*(kk-1) + (1:5);
            Aeq(rows, iX(:,kk+1)) = eye(5);                 %#ok<SPRIX>
            Aeq(rows, iX(:,kk))   = -Ak(:,:,kk);            %#ok<SPRIX>
            Aeq(rows, iU(:,kk))   = -Bk(:,:,kk);            %#ok<SPRIX>
            Aeq(rows, iS)         = -Sk(:,kk);              %#ok<SPRIX>
            Aeq(rows, iNu(:,kk))  = -eye(5);                %#ok<SPRIX>
            beq(rows) = ck(:,kk);
        end
        row = 5*K;
        Aeq(row + (1:5), iX(:,1)) = eye(5);  beq(row + (1:5)) = x0;
        row = row + 5;
        Aeq(row + (1:4), iX(1:4,N)) = eye(4); beq(row + (1:4)) = [xPad; 0; 0; 0];

        % Inequalities
        nIn = 10*K + K + 2*N + K;
        Ain = spalloc(nIn, nVar, 20*K + 4*N + 4*K);
        bin = zeros(nIn, 1);
        row = 0;
        for kk = 1:K
            for j = 1:5                                      % |nu| <= eta
                row = row + 1; Ain(row, [iNu(j,kk) iEta(j,kk)]) = [ 1 -1];   %#ok<SPRIX>
                row = row + 1; Ain(row, [iNu(j,kk) iEta(j,kk)]) = [-1 -1];   %#ok<SPRIX>
            end
        end
        for kk = 1:K                                         % pointing about vertical
            row = row + 1; Ain(row, [iU(2,kk) iU(3,kk)]) = [-1 cosTilt];     %#ok<SPRIX>
        end
        for kk = 1:N                                         % glide slope
            row = row + 1; Ain(row, [iX(1,kk) iX(2,kk)]) = [ 1 -cotGS]; bin(row) =  xPad;  %#ok<SPRIX>
            row = row + 1; Ain(row, [iX(1,kk) iX(2,kk)]) = [-1 -cotGS]; bin(row) = -xPad;  %#ok<SPRIX>
        end
        mu1 = rho1 * exp(-xb(5,1:K));
        mu2 = rho2 * exp(-xb(5,1:K));
        for kk = 1:K                                         % upper thrust (tangent)
            row = row + 1; Ain(row, [iU(3,kk) iX(5,kk)]) = [1 mu2(kk)];       %#ok<SPRIX>
            bin(row) = mu2(kk) * (1 + xb(5,kk));
        end

        % Cones: ||u|| <= sigma; lower thrust (2nd-order Taylor about zbar)
        cones = repmat(secondordercone(sparse(1, nVar), 0, sparse(nVar, 1), 0), 1, 2*K);
        for kk = 1:K
            Akk = sparse([1 2], iU(1:2,kk), [1 1], 2, nVar);
            cones(kk) = secondordercone(Akk, [0; 0], sparse(iU(3,kk), 1, 1, nVar, 1), 0);
            % mu1*(1 - dz + dz^2/2) <= sigma, dz = z - zbar
            % <=> (mu1/2) dz^2 <= L,  L = sigma + mu1*z - mu1*(1 + zbar)
            c0 = -mu1(kk) * (1 + xb(5,kk));
            aL = sparse([iU(3,kk) iX(5,kk)], 1, [1 mu1(kk)], nVar, 1);
            Akk = [sparse(1, iX(5,kk), sqrt(2*mu1(kk)), 1, nVar); aL'];
            bk = [sqrt(2*mu1(kk)) * xb(5,kk); 1 - c0];
            cones(K+kk) = secondordercone(Akk, bk, aL, -(c0 + 1));
        end

        % Bounds: physical + trust region (scaled units)
        lb = -inf(nVar, 1); ub = inf(nVar, 1);
        lb(iX) = xb(:) - rTR;  ub(iX) = xb(:) + rTR;
        lb(iU) = uRef(:) - rTR;  ub(iU) = uRef(:) + rTR;
        lb(iX(5,:)) = max(lb(iX(5,:)), zDry);  ub(iX(5,:)) = min(ub(iX(5,:)), 0);
        lb(iU(3,:)) = max(lb(iU(3,:)), 0);
        lb(iS) = max(sb - rTR, 0.05);  ub(iS) = sb + rTR;
        lb(iEta) = 0;

        f = zeros(nVar, 1);
        f(iX(5,N)) = -1;                 % maximize final mass
        f(iEta) = lambda;                % virtual-control penalty

        [xs, ~, flag, out] = coneprog(f, cones, Ain, bin, Aeq, beq, lb, ub, coneOpts);
        % exitflag -7 ("search direction too small") often comes with a
        % solution that is optimal to ~1e-10 but misses coneprog's own
        % stopping test: accept it when its reported residuals are tiny.
        ok = flag == 1 || (flag == -7 && out.primalfeasibility < 1e-6 ...
                           && out.dualfeasibility < 1e-6 && abs(out.dualitygap) < 1e-6);
        sol = struct('flag', flag);
        if ~ok, return, end
        sol.X = xs(iX);
        sol.U = xs(iU);
        sol.s = xs(iS);
        sol.nu = xs(iNu);
        sol.L = -sol.X(5,N) + lambda * sum(abs(sol.nu(:)));   % linearized cost
    end
end

% =====================================================================
function u = projectControl(u, z, rho1, rho2, maxTilt)
% Nearest-feasible control for the initial guess: tilt within maxTilt of
% vertical, magnitude within the thrust bounds at mass m0*e^z.
ang = atan2(u(1), u(2));                     % angle from vertical
if u(2) <= 0 || abs(ang) > maxTilt
    ang = sign(ang) * min(abs(ang), maxTilt);
end
mag = min(max(norm(u), rho1 * exp(-z)), rho2 * exp(-z));
u = [mag * sin(ang); mag * cos(ang); mag];
end

function F = dynamicsF(x, u, p, sc, alpha)
% Scaled dynamics per unit scaled time: x = [x y vx vy z] (km, Vsc, -),
% u = [ux uy sigma] (g). Drag uses the guidance model (exponential
% atmosphere, constant speed of sound, CD table), as guidance_model_eom.
vPhys = x(3:4) * sc.V;
V = norm(vPhys);
m = sc.m0 * exp(x(5));
rho = p.atmo.rho0 * exp(-max(x(2) * sc.L, 0) / p.atmo.H);
CD = interp1(p.aero.mach, p.aero.CD, min(V / p.atmo.soundSpeed, p.aero.mach(end)));
aDrag = -(0.5 * rho * V * CD * p.aero.A_ref / m) * vPhys;
F = [x(3:4); u(1:2) + [0; -1] + aDrag / sc.A; -alpha * u(3)];
end

function [Ak, Bk, Sk, ck] = discretize(xbar, ubar, sbar, model, dtau)
% Exact discretization of the dynamics linearized along the reference:
% integrate the reference state, the state-transition matrix Phi and the
% control/dilation sensitivities over each ZOH interval (RK4 sub-steps).
K = size(ubar, 2);
Ak = zeros(5,5,K); Bk = zeros(5,3,K); Sk = zeros(5,K); ck = zeros(5,K);
M = 4; h = dtau / M;
for k = 1:K
    x = xbar(:,k); u = ubar(:,k);
    Phi = eye(5); PsiB = zeros(5,3); PsiS = zeros(5,1);
    for j = 1:M
        [k1x, k1P, k1B, k1S] = augRHS(x, Phi, u, sbar, model);
        [k2x, k2P, k2B, k2S] = augRHS(x + h/2*k1x, Phi + h/2*k1P, u, sbar, model);
        [k3x, k3P, k3B, k3S] = augRHS(x + h/2*k2x, Phi + h/2*k2P, u, sbar, model);
        [k4x, k4P, k4B, k4S] = augRHS(x + h*k3x, Phi + h*k3P, u, sbar, model);
        x    = x    + h/6*(k1x + 2*k2x + 2*k3x + k4x);
        Phi  = Phi  + h/6*(k1P + 2*k2P + 2*k3P + k4P);
        PsiB = PsiB + h/6*(k1B + 2*k2B + 2*k3B + k4B);
        PsiS = PsiS + h/6*(k1S + 2*k2S + 2*k3S + k4S);
    end
    Ak(:,:,k) = Phi;
    Bk(:,:,k) = Phi * PsiB;
    Sk(:,k)   = Phi * PsiS;
    ck(:,k)   = x - Ak(:,:,k) * xbar(:,k) - Bk(:,:,k) * u - Sk(:,k) * sbar;
end
end

function [dx, dPhi, dPsiB, dPsiS] = augRHS(x, Phi, u, s, model)
% f = s*F(x,u):  A = s dF/dx,  B = s dF/du,  S = F  (central differences)
F = model(x, u);
A = zeros(5,5); B = zeros(5,3);
for i = 1:5
    e = zeros(5,1); e(i) = 1e-6;
    A(:,i) = (model(x + e, u) - model(x - e, u)) / 2e-6;
end
for i = 1:3
    e = zeros(3,1); e(i) = 1e-6;
    B(:,i) = (model(x, u + e) - model(x, u - e)) / 2e-6;
end
A = s * A; B = s * B;
dx = s * F;
dPhi = A * Phi;
PhiInv = Phi \ eye(5);
dPsiB = PhiInv * B;
dPsiS = PhiInv * F;
end

function xn = propagate(x, u, s, model, dtau)
M = 4; h = dtau / M;
for j = 1:M
    k1 = s*model(x, u); k2 = s*model(x + h/2*k1, u);
    k3 = s*model(x + h/2*k2, u); k4 = s*model(x + h*k3, u);
    x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
xn = x;
end

function d = defects(X, U, s, model, dtau)
K = size(U, 2);
d = zeros(5, K);
for k = 1:K
    d(:,k) = X(:,k+1) - propagate(X(:,k), U(:,k), s, model, dtau);
end
end

function J = nonlinearCost(X, U, s, model, dtau, lambda)
J = -X(5,end) + lambda * sum(abs(defects(X, U, s, model, dtau)), 'all');
end

function m = maxDefect(X, U, s, model, dtau)
m = max(abs(defects(X, U, s, model, dtau)), [], 'all');
end

function out = fly_landing_closed_loop(state0, p, opts)
%FLY_LANDING_CLOSED_LOOP Fly the landing burn in the plant with in-flight
%   re-planning (MATLAB reference implementation of what the Simulink
%   model does).
%
%   out = fly_landing_closed_loop(state0, p)
%     state0 - [x; y; vx; vy; m] at landing-burn ignition
%
%   Plan with landing_guidance_scvx at ignition; then every
%   p.landing.replanPeriod seconds re-solve from the measured state,
%   warm-started from the previous plan, as long as more than
%   p.landing.freezeTime of the plan remains and the stage is above
%   p.landing.minReplanAltitude (near the end the remaining horizon is too
%   short for a well-posed re-solve, so the last plan is flown to the end).
%   Commands are flown as thrust acceleration (T = aCmd * current mass).
%   After the plan ends the engine is off.
%
%   out fields: t, X, T, theta (trajectory from ignition), touchdown
%   (state at ground contact), nSolves, solveTimes, iterations, plans
%
%   Options: Verbose (one line per re-plan), MaxWallTime [s] (error out
%   instead of running indefinitely if something goes wrong).

arguments
    state0 (5,1) double
    p struct
    opts.Verbose (1,1) logical = false
    opts.MaxWallTime (1,1) double = 300
end

c = p.Isp * p.g0;
eom = @(s, a, th) threeDOF_EOM(0, s, @(~, x) deal(a * x(5), th), p);
gnd = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @touchdownEvent);

s = state0(:);
tNow = 0;
t = 0; X = s'; T = 0; TH = pi/2;
plan = landing_solve(s, p);
tPlan = 0;
solveTimes = NaN; iters = plan.iterations; plans = {plan};
tNextReplan = p.landing.replanPeriod;
landed = false;
wall = tic;

while ~landed
    if toc(wall) > opts.MaxWallTime
        error('fly_landing_closed_loop:timeout', ...
            'Exceeded %.0f s wall time at t = %.2f s (altitude %.1f m).', opts.MaxWallTime, tNow, s(2));
    end
    tLocal = tNow - tPlan;
    dtNode = plan.duration / numel(plan.t);
    remaining = plan.duration - tLocal;

    % Re-plan from the measured state (warm start from the current plan),
    % unless inside the final window where the plan is flown to the end
    canReplan = remaining > p.landing.freezeTime && s(2) > p.landing.minReplanAltitude;
    if canReplan && tNow >= tNextReplan - 1e-9
        ws = struct('X', plan.Xs, 'U', plan.Us, 's', plan.s, 'm0', plan.m0, 'elapsed', tLocal);
        tic;
        newPlan = landing_solve(s, p, ws);
        solveTimes(end+1) = toc; iters(end+1) = newPlan.iterations; %#ok<AGROW>
        if newPlan.converged
            plan = newPlan; tPlan = tNow; plans{end+1} = plan; %#ok<AGROW>
            tLocal = 0; dtNode = plan.duration / numel(plan.t);
        end
        tNextReplan = tNow + p.landing.replanPeriod;
        if opts.Verbose
            fprintf('  t %6.2f s  alt %7.1f m  speed %6.1f m/s  solve %.2f s (%d it)%s\n', ...
                tNow, s(2), norm(s(3:4)), solveTimes(end), iters(end), ...
                string(repmat(' NOT converged (kept old plan)', 1, ~newPlan.converged)));
        end
        remaining = plan.duration - tLocal;
        canReplan = remaining > p.landing.freezeTime && s(2) > p.landing.minReplanAltitude;
    end

    % Fly to the next node boundary, the end of the plan, or -- only while
    % re-planning is still possible -- the next re-plan time. (Bug fixed:
    % the re-plan time used to bound the step even inside the final window,
    % where it is no longer advanced, giving 1e-6 s steps for seconds.)
    if tLocal < plan.duration - 1e-9
        k = min(floor(tLocal / dtNode + 1e-9) + 1, numel(plan.t));
        tStop = plan.duration;
        if canReplan, tStop = min(tStop, tNextReplan - tPlan); end
        tEnd = min(k * dtNode, tStop);
        a = plan.aCmd(k); th = plan.theta(k);
    else
        tEnd = tLocal + 10; a = 0; th = pi/2;        % plan over: engine off
    end
    if tEnd <= tLocal + 1e-9
        error('fly_landing_closed_loop:noProgress', ...
            'Step would not advance time (t = %.4f s, local %.4f s, end %.4f s).', tNow, tLocal, tEnd);
    end
    [tt, Xk, te] = ode45(@(~, x) eom(x, a, th), [tLocal tEnd], s, gnd);
    s = Xk(end,:)';
    t = [t; tPlan + tt(2:end)];                          %#ok<AGROW>
    X = [X; Xk(2:end,:)];                                %#ok<AGROW>
    T = [T; a * Xk(2:end,5)];                            %#ok<AGROW>
    TH = [TH; th * ones(numel(tt) - 1, 1)];              %#ok<AGROW>
    tNow = tPlan + tt(end);
    landed = ~isempty(te);
end

out = struct('t', t, 'X', X, 'T', T, 'theta', TH, 'touchdown', s, ...
    'nSolves', numel(iters), 'solveTimes', solveTimes, 'iterations', iters, ...
    'plans', {plans}, 'propUsed', state0(5) - s(5));
end

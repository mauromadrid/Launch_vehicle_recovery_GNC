function plan = landing_solve(state, p, warmStart)
%LANDING_SOLVE One landing-guidance solve as flown in flight: plan with a
%   thrust margin, fall back to the full engine thrust if needed.
%
%   plan = landing_solve(state, p)              % first plan (at ignition)
%   plan = landing_solve(state, p, warmStart)   % re-plan, warm-started
%
%   Plans with p.landing.thrustPlanFraction of T_max. A fuel-optimal plan
%   rides its upper thrust bound at the end of the burn; planning below the
%   real maximum keeps throttle authority in reserve for later re-plans to
%   absorb model errors. If no plan exists within that margin any more, it
%   re-solves with the full T_max (plan.usedFullThrust = true).
%
%   Shared by fly_landing_closed_loop (MATLAB) and landing_replan (the
%   Simulink model's guidance block), so both fly the same logic.

if nargin < 3, warmStart = []; end
plan = landing_guidance_scvx(state(:), p, N=p.landing.N, WarmStart=warmStart, ...
                             ThrustFraction=p.landing.thrustPlanFraction);
plan.usedFullThrust = false;
if ~plan.converged && p.landing.thrustPlanFraction < 1
    plan = landing_guidance_scvx(state(:), p, N=p.landing.N, WarmStart=warmStart, ThrustFraction=1);
    plan.usedFullThrust = true;
end
end

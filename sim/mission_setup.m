function [bb, ld] = mission_setup(p)
%MISSION_SETUP Pre-flight guidance computations for a parameter set: what
%   the flight software has to compute before the simulation can start.
%
%   [bb, ld] = mission_setup(p)
%     bb - boostback plan, aimed for drag + entry burn (boostback_targeting)
%     ld - landing-burn profile (placeholder until Phase 3)
%
%   Single definition shared by the Simulink model's InitFcn (interactive
%   runs) and simulate_mission (programmatic runs, Monte Carlo), so both
%   always fly the same mission logic. The entry burn is not here: it's
%   sized in flight, at its start altitude, inside the model.

bb = boostback_targeting([], p);
ld = pdg_landing_guidance([], p);
end

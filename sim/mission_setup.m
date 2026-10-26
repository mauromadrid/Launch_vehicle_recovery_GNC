function bb = mission_setup(p)
%MISSION_SETUP Pre-flight guidance computations for a parameter set: what
%   the flight software has to compute before the simulation can start.
%
%   bb = mission_setup(p)
%     bb - boostback plan, aimed for drag + entry burn (boostback_targeting)
%
%   Single definition shared by the Simulink model's InitFcn (interactive
%   runs) and simulate_mission (programmatic runs, Monte Carlo), so both
%   always fly the same mission logic. The entry burn and the landing burn
%   are not here: both are computed in flight, inside the model, from the
%   measured state (entry: once at its start altitude; landing: at ignition
%   and then re-planned periodically).

bb = boostback_targeting([], p);
end

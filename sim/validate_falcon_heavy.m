%% VALIDATE_FALCON_HEAVY
% Sanity check of the boostback guidance at real scale: Falcon Heavy side
% boosters returning to LZ-1/LZ-2 (see params/falconHeavySideBoosterParams.m
% for what is sourced vs. assumed).
%
% BECO state and propellant at BECO are not public, so this sweeps a
% plausible envelope and checks three things:
%   1. Feasibility is monotonic in propellant: more propellant must never
%      turn a feasible boostback infeasible (a solver/formulation bug).
%   2. The minimum propellant each BECO state needs for the boostback.
%   3. The predicted boostback duration vs. the observed "just over one
%      minute" (NASA Roman launch blog, 2026-08-30).
% Model limits: planar, flat-Earth, vacuum -- fine for a boostback at
% 60-120 km, and the reason entry/landing burns are out of scope here.

clear; clc;
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

% ---- BECO envelope (assumed, not sourced) ----
speeds  = [1500 1700 1900];   % [m/s] speed at BECO
fpaDeg  = [25 35];            % [deg] flight-path angle at BECO
alt     = 60e3;               % [m]
xDown   = 50e3;               % [m] downrange of the pad
propGrid = (10:10:80) * 1e3;  % [kg] propellant at BECO, for the monotonicity check

results = table();
for V = speeds
    for g = fpaDeg
        sep = struct('x', xDown, 'y', alt, 'vx', V*cosd(g), 'vy', V*sind(g));

        % 1. Feasibility over the propellant grid (+ duration where feasible)
        feasible = false(size(propGrid));
        tb = nan(size(propGrid));
        for i = 1:numel(propGrid)
            [feasible(i), bb] = tryBoostback(sep, propGrid(i));
            if feasible(i), tb(i) = bb.duration; end
        end
        monotonic = ~any(diff(feasible) < 0);

        % 2. Minimum propellant: bisection between the last infeasible and
        %    first feasible grid points (valid only if monotonic)
        propMin = NaN;
        iFirst = find(feasible, 1);
        if monotonic && ~isempty(iFirst) && iFirst > 1
            lo = propGrid(iFirst - 1); hi = propGrid(iFirst);
            while hi - lo > 250
                mid = (lo + hi) / 2;
                if tryBoostback(sep, mid), hi = mid; else, lo = mid; end
            end
            propMin = hi;
        elseif iFirst == 1
            propMin = propGrid(1); % upper bound only
        end

        % 3. Duration at the minimum and at 1.5x the minimum propellant
        [~, bbMin] = tryBoostback(sep, propMin);
        [~, bbMargin] = tryBoostback(sep, 1.5 * propMin);

        results = [results; table(V, g, monotonic, propMin/1e3, ...
            bbMin.duration, bbMargin.duration, bbMargin.propUsed/1e3, ...
            'VariableNames', {'V_BECO', 'FPA_deg', 'Monotonic', 'PropMin_t', ...
            'Burn_s_atMin', 'Burn_s_at1p5xMin', 'PropUsed_t_at1p5xMin'})]; %#ok<AGROW>
        fprintf('V=%4d m/s, FPA=%2d deg: feasibility over %d-%d t: %s\n', ...
                V, g, propGrid(1)/1e3, propGrid(end)/1e3, mat2str(feasible));
    end
end

fprintf('\nFull propellant load per core ~410.9 t (433.1 t fueled - 22.2 t dry).\n');
fprintf('Observed boostback duration: "just over one minute".\n\n');
disp(results);

function [ok, bb] = tryBoostback(sep, prop)
bb = struct('duration', NaN, 'propUsed', NaN);
ok = false;
if ~isfinite(prop), return; end
p = falconHeavySideBoosterParams(sep, prop);
try
    bb = boostback_guidance([], p);
    ok = true;
catch err
    if ~strcmp(err.identifier, 'boostback_guidance:infeasible'), rethrow(err); end
end
end

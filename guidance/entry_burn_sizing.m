function eb = entry_burn_sizing(state, p)
%ENTRY_BURN_SIZING Minimum entry-burn delta-v that meets the descent limits.
%
%   eb = entry_burn_sizing(state, p)
%     state - measured [x; y; vx; vy; m] at burn start (descending through
%             p.entry.startAltitude)
%
%   Numerical predictor-corrector, run once at burn start:
%     predict  - fly the burn (full thrust, retrograde) + the unpowered
%                descent to landing-burn ignition altitude through the
%                GUIDANCE model (guidance_model_eom: exponential
%                atmosphere, constant speed of sound, the CD(Mach) table).
%                Deliberately simpler than the plant (COESA), as onboard
%                models are.
%     correct  - for each active limit, root-find (fzero) the smallest
%                delta-v whose predicted metric meets it:
%                  peak dynamic pressure        <= p.entry.qMax
%                  peak heating index sqrt(rho)V^3 <= p.entry.heatMaxRel x
%                                                  (its no-burn value)
%                  speed at ignition altitude   <= p.entry.vIgnitionMax
%                every metric decreases monotonically with delta-v, so a
%                bracketed 1D root find is reliable. The absolute limits
%                (q, ignition speed) are tightened by p.entry.limitMargin
%                to cover the guidance model's known under-prediction.
%   The burn delta-v is the largest of those (all limits met).
%
%   Why size the burn to the limits and not lower: speed removed high up
%   is speed drag would largely have removed for free, so the minimum
%   burn that keeps the descent within limits is the propellant-optimal
%   one for a fixed start altitude.
%
%   eb fields:
%     dv          - commanded delta-v [m/s] (0 = no burn needed)
%     mCutoff     - mass at which to cut the engine: m*exp(-dv/(Isp*g0))
%     propUsed    - planned propellant [kg]
%     feasible    - all limits achievable within the propellant allowed
%                   (above dryMass + landingPropReserve) and without
%                   removing more than the current speed
%     binding     - which limit set dv ("qMax", "heatMaxRel",
%                   "vIgnitionMax", or "none")
%     dvByLimit   - struct: dv each limit alone would need (NaN = can't)
%     predicted   - predictor metrics at the chosen dv (qPeak, heatPeak,
%                   heatLoad, vIgnition, final = state at ignition)
%     noBurn      - predictor metrics with dv = 0

c = p.Isp * p.g0;
m0 = state(5);
mMin = p.dryMass + p.landingPropReserve;
% Upper bound on the burn: the propellant allowed (keeping the landing
% reserve), and never more than the current speed -- a retrograde burn
% can't usefully remove more speed than there is, and driving V through
% zero makes "opposite to the velocity" flip direction every step.
dvMax = max(min(c * log(m0 / mMin), norm(state(3:4))), 0);

% Design margin on the absolute limits: the guidance model (exponential
% atmosphere) under-predicts the plant's peak q by ~3-8%, so sizing to the
% limit itself left almost every burned case above it. Size to
% limit/(1 + margin) instead. The heating limit is relative to this same
% model's no-burn prediction, so the model bias largely cancels there and
% no margin is applied.
margin = 0;
if isfield(p.entry, 'limitMargin'), margin = p.entry.limitMargin; end

noBurn = predictDescent(state, 0, p);
limits = struct( ...
    'qMax',         struct('field', 'qPeak',    'value', p.entry.qMax / (1 + margin)), ...
    'heatMaxRel',   struct('field', 'heatPeak', 'value', p.entry.heatMaxRel * noBurn.heatPeak), ...
    'vIgnitionMax', struct('field', 'vIgnition','value', p.entry.vIgnitionMax / (1 + margin)));

names = fieldnames(limits);
dvByLimit = struct();
feasible = true;
for i = 1:numel(names)
    L = limits.(names{i});
    if ~isfinite(L.value)
        dvByLimit.(names{i}) = 0;          % inactive limit
        continue
    end
    excess = @(dv) predictDescent(state, dv, p).(L.field) - L.value;
    if excess(0) <= 0
        dvByLimit.(names{i}) = 0;          % already met without a burn
    elseif dvMax > 0 && excess(dvMax) <= 0
        % NB: fzero's TolX is *relative* to |x| (0.5 would accept +-50%)
        dvByLimit.(names{i}) = fzero(excess, [0, dvMax], optimset('TolX', 1e-4));
    else
        dvByLimit.(names{i}) = NaN;        % not achievable with the propellant allowed
        feasible = false;
    end
end

dvs = cellfun(@(n) dvByLimit.(n), names);
if feasible
    [dv, iMax] = max(dvs);
    binding = "none";
    if dv > 0, binding = string(names{iMax}); end
else
    dv = dvMax;                            % best effort: all allowed propellant
    binding = strjoin(string(names(isnan(dvs))), ",");
end

eb.dv        = dv;
eb.mCutoff   = m0 * exp(-dv / c);
eb.propUsed  = m0 - eb.mCutoff;
eb.feasible  = feasible;
eb.binding   = binding;
eb.dvByLimit = dvByLimit;
eb.predicted = predictDescent(state, dv, p);
eb.noBurn    = noBurn;
end

% =====================================================================
function out = predictDescent(state, dv, p)
% Guidance-model prediction: burn dv retrograde at full thrust, then fall
% to ignition altitude. Returns peak q, peak heating index, heat load and
% speed at ignition altitude.
c = p.Isp * p.g0;
mCut = state(5) * exp(-dv / c);
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-6, 'MaxStep', 0.5);

t = 0; X = state(:)';
if dv > 0
    burnOpts = odeset(opts, 'Events', @(~, s) massEvent(s, mCut));
    [t, X] = ode45(@(~, s) guidance_model_eom(s, p.T_max_entry, p), [0 600], state, burnOpts);
end
coastOpts = odeset(opts, 'Events', @(~, s) altEvent(s, p.landingIgniteAltitude));
[t2, X2] = ode45(@(~, s) guidance_model_eom(s, 0, p), t(end) + [0 1200], X(end,:)', coastOpts);
t = [t; t2(2:end)]; X = [X; X2(2:end,:)];

rho = p.atmo.rho0 * exp(-max(X(:,2), 0) / p.atmo.H);
V = vecnorm(X(:,3:4), 2, 2);
heat = sqrt(rho) .* V.^3;
out.qPeak     = max(0.5 * rho .* V.^2);
out.heatPeak  = max(heat);
out.heatLoad  = trapz(t, heat);
out.vIgnition = V(end);
out.final     = X(end,:)';         % predicted state at ignition altitude
end

function [value, isterminal, direction] = massEvent(s, mCut)
value = s(5) - mCut; isterminal = 1; direction = -1;
end

function [value, isterminal, direction] = altEvent(s, h)
value = s(2) - h; isterminal = 1; direction = -1;
end

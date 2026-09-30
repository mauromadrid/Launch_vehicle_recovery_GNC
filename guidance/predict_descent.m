function pred = predict_descent(stateCutoff, p)
%PREDICT_DESCENT Guidance-model prediction of the whole descent after the
%   boostback: coast to the entry-burn start altitude, the entry burn the
%   flight computer would size there (entry_burn_sizing), and the coast to
%   landing-burn ignition altitude.
%
%   pred = predict_descent(stateCutoff, p)
%     stateCutoff - [x; y; vx; vy; m] at boostback cutoff
%     pred.final  - predicted state at ignition altitude
%     pred.entry  - entry_burn_sizing result ([] if no entry burn)
%
%   Uses guidance_model_eom (exponential atmosphere), never the plant: this
%   is what the boostback targeting can afford to know.

opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-6, 'MaxStep', 1);
pred.entry = [];

hStop = p.landingIgniteAltitude;
useEntry = isfield(p, 'entry') && p.entry.enabled;
if useEntry
    hStop = p.entry.startAltitude;
end
[~, X] = ode45(@(~, s) guidance_model_eom(s, 0, p), [0 3000], stateCutoff(:), ...
               odeset(opts, 'Events', @(~, s) altEvent(s, hStop)));

reachedStart = useEntry && abs(X(end,2) - hStop) < 1 && X(end,4) < 0;
if reachedStart
    pred.entry = entry_burn_sizing(X(end,:)', p);
    pred.final = pred.entry.predicted.final;
elseif useEntry
    % apogee below the entry start altitude: plain coast to ignition
    [~, X] = ode45(@(~, s) guidance_model_eom(s, 0, p), [0 3000], stateCutoff(:), ...
                   odeset(opts, 'Events', @(~, s) altEvent(s, p.landingIgniteAltitude)));
    pred.final = X(end,:)';
else
    pred.final = X(end,:)';
end
end

function [value, isterminal, direction] = altEvent(s, h)
value = s(2) - h; isterminal = 1; direction = -1;
end

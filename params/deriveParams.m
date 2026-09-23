function p = deriveParams(p)
%DERIVEPARAMS Recompute every parameter that is derived from others.
%   Call after changing any base parameter (engine thrust, engine counts,
%   throttle limit, dry/propellant mass), e.g. after applying Monte Carlo
%   dispersions, so derived values can never go stale.

% Thrust bounds per burn: an engine that is lit can't throttle below
% throttle_min, so the lower bound scales with the engine count too.
p.T_max = p.nEnginesLanding * p.T_engine_max;       % landing burn
p.T_min = p.throttle_min * p.T_max;
p.T_max_boostback = p.nEnginesBoostback * p.T_engine_max;
p.T_min_boostback = p.throttle_min * p.T_max_boostback;

p.m0 = p.dryMass + p.propMassAtSeparation;
end

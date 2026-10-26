function rho = guidance_density(h, p)
%GUIDANCE_DENSITY Onboard (guidance-model) atmospheric density [kg/m^3].
%
%   rho = guidance_density(h, p)      h: altitude [m], any size
%
%   Onboard atmosphere table: COESA 1976 density sampled every 5 km from 0
%   to 80 km (p.atmo.tableAltitude, p.atmo.tableLogRho), interpolated
%   linearly in log(density) -- i.e. piecewise exponential -- and
%   extrapolated the same way above 80 km. Within 0.8 % of COESA below
%   5 km and 2.5 % overall: close, but deliberately not the plant's model.
%
%   History: v1 used a single exponential fit over 0-80 km (rho0, H),
%   which was 10-16 % too dense in the last 3 km. That error made the
%   landing burn arrive faster than planned (9.5 m/s touchdowns, then
%   infeasible re-plans), and caused the systematic boostback-targeting
%   miss and the entry burn's q under-prediction.

rho = exp(interp1(p.atmo.tableAltitude, p.atmo.tableLogRho, max(h, 0), 'linear', 'extrap'));
end

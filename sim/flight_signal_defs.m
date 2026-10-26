function defs = flight_signal_defs()
%FLIGHT_SIGNAL_DEFS Catalog of the signals stored in a flight record.
%
%   defs = flight_signal_defs()
%
%   One entry per signal. To add a variable (new DoF, control, load...),
%   add one entry here: make_flight_record computes and saves it, and
%   plot_flight_record can plot it by name. Nothing else needs to change.
%
%   Fields of each entry:
%     name          column name in the record (valid MATLAB identifier)
%     description   label text
%     unit          stored (SI) unit
%     displayUnit   unit shown in plots;  displayScale  stored * scale = shown
%     compute       @(raw, aux, p) -> n x 1 column (SI)
%     lower, upper  [] (no limit) or @(raw, aux, p) -> scalar or n x 1 column
%     lowerLabel, upperLabel  legend text for the limit lines
%     plotStyle     "line" or "stairs" (piecewise-constant commands)
%
%   raw: t, X = [x y vx vy m], T (thrust), theta (thrust angle), ev (events)
%   aux: helper quantities computed once (rho, mach, q, heatIdx, aDrag,
%        thrust limits per phase, phase) -- see make_flight_record

S = @(name, desc, unit, dUnit, dScale, fcn) struct('name', name, 'description', desc, ...
    'unit', unit, 'displayUnit', dUnit, 'displayScale', dScale, 'compute', fcn, ...
    'lower', [], 'upper', [], 'lowerLabel', "", 'upperLabel', "", 'plotStyle', "line");

defs = [
    S("x",        "Downrange position",                 "m",   "km",  1e-3, @(r,a,p) r.X(:,1))
    S("altitude", "Altitude",                           "m",   "km",  1e-3, @(r,a,p) r.X(:,2))
    S("vx",       "Downrange velocity",                 "m/s", "m/s", 1,    @(r,a,p) r.X(:,3))
    S("vy",       "Vertical velocity",                  "m/s", "m/s", 1,    @(r,a,p) r.X(:,4))
    S("mass",     "Mass",                               "kg",  "kg",  1,    @(r,a,p) r.X(:,5))
    S("speed",    "Speed",                              "m/s", "m/s", 1,    @(r,a,p) vecnorm(r.X(:,3:4), 2, 2))
    S("mach",     "Mach number",                        "",    "",    1,    @(r,a,p) a.mach)
    S("thrust",   "Thrust",                             "N",   "kN",  1e-3, @(r,a,p) r.T)
    S("thrustAngle", "Thrust angle (from +x, downrange)", "rad", "deg", 180/pi, ...
        @(r,a,p) thrustAngle(r))
    S("loadFactor", "Inertial (sensed) acceleration",    "m/s^2", "g", 1/9.80665, ...
        @(r,a,p) vecnorm(r.T ./ r.X(:,5) .* [cos(r.theta) sin(r.theta)] + a.aDrag, 2, 2))
    S("dynPressure", "Dynamic pressure",                "Pa",  "kPa", 1e-3, @(r,a,p) a.q)
    S("heatIndex", "Heating index sqrt(rho) V^3",       "kg^0.5 m^1.5 s^-3", "rel.", 1, @(r,a,p) a.heatIdx)
    S("heatFlux", "Stagnation heat flux (Sutton-Graves est.)", "W/m^2", "kW/m^2", 1e-3, ...
        @(r,a,p) 1.7415e-4 * sqrt(a.rho / p.aero.effectiveNoseRadius) .* vecnorm(r.X(:,3:4), 2, 2).^3)
    ];

% ---- Limits ----
k = find([defs.name] == "thrust");
defs(k).lower = @(r,a,p) a.thrustMin;  defs(k).lowerLabel = "T_{min}";
defs(k).upper = @(r,a,p) a.thrustMax;  defs(k).upperLabel = "T_{max}";
defs(k).plotStyle = "stairs";

k = find([defs.name] == "dynPressure");
defs(k).upper = @(r,a,p) qLimit(p);    defs(k).upperLabel = "q limit (entry burn)";

k = find([defs.name] == "heatIndex");
defs(k).upper = @(r,a,p) heatLimit(a, p); defs(k).upperLabel = "heating limit";

k = find([defs.name] == "thrustAngle");
defs(k).plotStyle = "stairs";
end

% =====================================================================
function th = thrustAngle(r)
% Undefined while the engine is off: NaN, so plots leave a gap
th = r.theta;
th(r.T <= 0) = NaN;
end

function q = qLimit(p)
q = NaN;
if isfield(p, 'entry') && p.entry.enabled && isfinite(p.entry.qMax)
    q = p.entry.qMax;
end
end

function h = heatLimit(a, p)
% The entry-burn heating limit is relative to the no-burn peak heating
% index; an absolute line needs that reference (aux.heatIdxNoBurnPeak).
h = NaN;
if isfield(p, 'entry') && p.entry.enabled && isfinite(p.entry.heatMaxRel) ...
        && isfield(a, 'heatIdxNoBurnPeak') && isfinite(a.heatIdxNoBurnPeak)
    h = p.entry.heatMaxRel * a.heatIdxNoBurnPeak;
end
end

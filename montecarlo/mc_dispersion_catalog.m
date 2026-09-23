function catalog = mc_dispersion_catalog(p)
%MC_DISPERSION_CATALOG All variables the Monte Carlo framework knows how to
%   disperse, with default distributions centred on the nominal values of
%   the given parameter set p (so the same catalog works for the small
%   illustrative vehicle and for falconHeavySideBoosterParams).
%
%   Pick the variables for a study with mc_select(catalog, names). Edit a
%   row's distribution directly in the returned table, or append rows for
%   any other numeric field of p using its dotted path as the Name (e.g.
%   "target.x", "landingIgniteAltitude") -- mc_apply_sample handles those
%   generically.
%
%   Columns:
%     Name         - "speed" / "fpa" (derived, see below) or a dotted path
%                    into p, e.g. "sep.y", "Isp"
%     Distribution - "uniform": A = lower, B = upper bound
%                    "normal":  A = mean,  B = standard deviation
%                    (normal samples are truncated at +-3 sigma: physical
%                    parameters don't have unbounded tails, and a single
%                    5-sigma outlier shouldn't dominate a small sample)
%     Units, Description
%
%   Derived variables: "speed" [m/s] and "fpa" [deg] are the polar form of
%   the separation velocity (p.sep.vx, p.sep.vy). They're provided because
%   that's how trajectory dispersions are normally specified -- a slower
%   or more lofted ascent -- rather than as independent vx/vy errors.
%
%   Defaults are uniform, i.e. set up for *sensitivity / feasibility
%   mapping* (which region of the input space works?). For Phase 5
%   statistics (how often does it work?), switch rows to "normal" with
%   realistic 1-sigma values.

V0   = hypot(p.sep.vx, p.sep.vy);
fpa0 = atan2d(p.sep.vy, p.sep.vx);

rows = {
%  Name                    Dist       A                              B                              Units   Description
  "speed",                 "uniform", 0.85*V0,                       1.15*V0,                       "m/s",  "Separation speed"
  "fpa",                   "uniform", 5,                             45,                            "deg",  "Separation flight-path angle (0 = horizontal)"
  "sep.x",                 "uniform", p.sep.x - 10e3,                p.sep.x + 10e3,                "m",    "Separation downrange position"
  "sep.y",                 "uniform", p.sep.y - 10e3,                p.sep.y + 10e3,                "m",    "Separation altitude"
  "propMassAtSeparation",  "uniform", 0.9*p.propMassAtSeparation,    1.1*p.propMassAtSeparation,    "kg",   "Propellant remaining at separation"
  "dryMass",               "uniform", 0.95*p.dryMass,                1.05*p.dryMass,                "kg",   "Stage dry mass"
  "Isp",                   "uniform", 0.98*p.Isp,                    1.02*p.Isp,                    "s",    "Specific impulse"
  "T_engine_max",          "uniform", 0.97*p.T_engine_max,           1.03*p.T_engine_max,           "N",    "Max thrust per engine"
};
catalog = cell2table(rows, 'VariableNames', ...
    {'Name', 'Distribution', 'A', 'B', 'Units', 'Description'});
catalog.Nominal = arrayfun(@(n) nominalValue(p, n, V0, fpa0), catalog.Name);
catalog = movevars(catalog, 'Nominal', 'Before', 'Distribution');
end

function v = nominalValue(p, name, V0, fpa0)
switch name
    case "speed", v = V0;
    case "fpa",   v = fpa0;
    otherwise
        path = strsplit(name, '.');
        v = getfield(p, path{:});
end
end

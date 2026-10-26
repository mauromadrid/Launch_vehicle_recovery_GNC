function L = mc_labels(name, results)
%MC_LABELS Display name, unit and display scale for a Monte Carlo column.
%
%   L = mc_labels(name, results)
%     L.text  - axis label, e.g. "Peak dynamic pressure [kPa]"
%     L.short - name without unit, for compact labels
%     L.scale - multiply stored (SI) values by this for display
%
%   Inputs (dispersed variables) are described by the dispersion catalog
%   stored with the results (UserData.spec: Description + Units); outputs
%   (metrics) by the table below. Unknown names fall back to the raw name,
%   so new metrics still plot, just without a nice label.

persistent metrics
if isempty(metrics)
    %            name              description                                   unit    scale
    rows = {
        "feasible",        "Boostback feasible (1 = yes)",                     "",      1
        "propBoostback",   "Boostback propellant",                             "kg",    1
        "propEntry",       "Entry-burn propellant",                            "kg",    1
        "landingMargin",   "Landing propellant margin (left minus ideal need)", "kg",   1
        "propAtIgnition",  "Propellant left at landing-burn ignition",         "kg",    1
        "qPeak",           "Max dynamic pressure (max-Q)",                     "kPa",   1e-3
        "maxQAltitude",    "Altitude of max-Q",                                "km",    1e-3
        "maxQMach",        "Mach number at max-Q",                             "",      1
        "nAxialMax",       "Peak axial (sensed) acceleration",                 "g",     1
        "nAxialBoostback", "Peak axial acceleration, boostback",               "g",     1
        "nAxialDescent",   "Peak axial acceleration, descent",                 "g",     1
        "heatFluxMax",     "Peak stagnation heat flux (Sutton-Graves est.)",   "kW/m^2", 1e-3
        "heatLoad",        "Integrated heat load (Sutton-Graves est.)",        "MJ/m^2", 1e-6
        "heatPeak",        "Peak heating index sqrt(rho) V^3",                 "rel.",  1
        "entryLimitMet",   "Peak q within limit (1 = yes)",                    "",      1
        "entrySized",      "Entry burn flown (1 = yes)",                       "",      1
        "apogee",          "Apogee after boostback",                           "km",    1e-3
        "ignitionSpeed",   "Speed at landing-burn ignition",                   "m/s",   1
        "ignitionMissX",   "Downrange miss at landing-burn ignition",          "m",     1
        "aimBias",         "Boostback aim-point bias",                         "m",     1
        "burnTime",        "Boostback burn duration",                          "s",     1
        "touchdownSpeed",  "Speed at touchdown",                               "m/s",   1
        "touchdownVx",     "Horizontal velocity at touchdown",                 "m/s",   1
        "touchdownVy",     "Vertical velocity at touchdown",                   "m/s",   1
        "touchdownMissX",  "Downrange miss at touchdown",                      "m",     1
        "propAtTouchdown", "Propellant left at touchdown",                     "kg",    1
        "nLandingPlans",   "Landing plans made in flight",                     "",      1
        "simTime",         "Simulink run time",                                "s",     1
        };
    metrics = cell2struct(rows, {'name', 'desc', 'unit', 'scale'}, 2);
end

name = string(name);
L = struct('short', name, 'unit', "", 'scale', 1);

% Dispersed inputs: from the catalog saved with the results
ud = results.Properties.UserData;
if isfield(ud, 'spec') && any(ud.spec.Name == name)
    row = ud.spec(ud.spec.Name == name, :);
    L.short = row.Description;
    L.unit = row.Units;
else
    k = find([metrics.name] == name, 1);
    if ~isempty(k)
        L.short = metrics(k).desc;
        L.unit = metrics(k).unit;
        L.scale = metrics(k).scale;
    end
end

L.text = L.short;
if strlength(L.unit) > 0
    L.text = L.short + " [" + L.unit + "]";
end
end

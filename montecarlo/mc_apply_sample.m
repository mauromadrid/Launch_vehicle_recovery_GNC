function p = mc_apply_sample(p, sample)
%MC_APPLY_SAMPLE Return the parameter set for one Monte Carlo case: the
%   nominal p with the dispersed values of one sample row applied.
%
%   p      - nominal parameter struct (vehicleParams or a variant)
%   sample - one row of the table from mc_generate_samples
%
%   Column names are either a derived variable ("speed", "fpa") or a
%   dotted path into p ("sep.y", "Isp", ...). Derived parameters (m0,
%   thrust bounds) are recomputed afterwards via deriveParams, so e.g.
%   dispersing dryMass correctly changes the separation mass.

names = string(sample.Properties.VariableNames);

% Separation velocity in polar form: speed and flight-path angle. If only
% one is dispersed, the other keeps its nominal value.
V   = hypot(p.sep.vx, p.sep.vy);
fpa = atan2d(p.sep.vy, p.sep.vx);
if ismember("speed", names), V   = sample.speed; end
if ismember("fpa",   names), fpa = sample.fpa;   end
p.sep.vx = V * cosd(fpa);
p.sep.vy = V * sind(fpa);

for name = setdiff(names, ["speed", "fpa"])
    path = strsplit(name, '.');
    if ~isfield_path(p, path)
        error('mc_apply_sample:unknownField', ...
            'Dispersion "%s" does not match any field of the parameter struct.', name);
    end
    p = setfield(p, path{:}, sample.(name));
end

p = deriveParams(p);
end

function tf = isfield_path(s, path)
tf = true;
for k = 1:numel(path)
    if ~isstruct(s) || ~isfield(s, path{k})
        tf = false;
        return
    end
    s = s.(path{k});
end
end

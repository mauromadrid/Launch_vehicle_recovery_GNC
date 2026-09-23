function spec = mc_select(catalog, names)
%MC_SELECT Pick the variables to disperse in a study, by name, from a
%   dispersion catalog (mc_dispersion_catalog). Errors on unknown names
%   rather than silently dispersing nothing.
%
%   spec = mc_select(catalog, ["speed", "fpa"])

names = string(names);
unknown = setdiff(names, catalog.Name);
if ~isempty(unknown)
    error('mc_select:unknownVariable', ...
        'Unknown dispersion variable(s): %s. Available: %s', ...
        strjoin(unknown, ', '), strjoin(catalog.Name, ', '));
end
[~, idx] = ismember(names, catalog.Name);
spec = catalog(idx, :);
end

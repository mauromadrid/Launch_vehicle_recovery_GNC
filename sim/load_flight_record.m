function [rec, file] = load_flight_record(file)
%LOAD_FLIGHT_RECORD Load a saved flight record.
%
%   rec = load_flight_record(file)
%   rec = load_flight_record()        % most recent file in results/flights
%   rec = load_flight_record("name")  % most recent file starting with "name"

folder = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'flights');
if nargin < 1 || (isstring(file) || ischar(file)) && ~isfile(file)
    prefix = "";
    if nargin >= 1, prefix = string(file); end
    d = dir(fullfile(folder, prefix + "*.mat"));
    if isempty(d)
        error('load_flight_record:noFiles', 'No flight records matching "%s*" in %s.', prefix, folder);
    end
    [~, k] = max([d.datenum]);
    file = fullfile(d(k).folder, d(k).name);
end
S = load(file, 'rec');
rec = S.rec;
end

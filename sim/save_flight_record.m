function file = save_flight_record(rec, folder)
%SAVE_FLIGHT_RECORD Save a flight record to results/flights (default).
%
%   file = save_flight_record(rec)
%   file = save_flight_record(rec, folder)
%
%   File name: <rec.meta.name>_<yyyyMMdd_HHmmss>.mat, containing the
%   variable "rec". Load with load_flight_record, plot with
%   plot_flight_record. results/ is not version-controlled.

if nargin < 2
    folder = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'flights');
end
if ~exist(folder, 'dir'), mkdir(folder); end
stamp = string(datetime(rec.meta.created, 'Format', 'yyyyMMdd_HHmmss'));
file = fullfile(folder, rec.meta.name + "_" + stamp + ".mat");
save(file, 'rec');
end

%% RUN_MC_MISSION
% Monte Carlo / sensitivity study of the whole mission: every sample is
% flown through the main simulation (the Simulink model, via
% simulate_mission -> mc_case_mission).
%
% Pipeline (each step is its own function):
%   mc_dispersion_catalog  what *can* be dispersed, with default ranges
%   mc_select              what *is* dispersed in this study
%   mc_generate_samples    N input sets (Latin hypercube, seeded)
%   mc_run                 one case per sample via a case function
%                          (mc_case_mission: simulate_mission + metrics),
%                          errors captured
%   mc_analyze             statistics, sensitivity heatmap, plots
%
% Results are saved to results/ (not version-controlled) so a long run can
% be re-analysed without re-running it:
%   S = load('results/mc_mission_<timestamp>.mat'); mc_analyze(S.results, ...)
%
% Cost: ~14 s per case (targeting + a Simulink run), so N = 200 is ~45 min.
% Run it in a separate process to keep the interactive session free:
%   matlab -batch "run('montecarlo/run_mc_mission.m')"

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projRoot));

%% ---- Study configuration ----
% Defaults; any of these can be set by the caller before running the
% script (e.g. a batch wrapper studying another landing site).
if ~exist('pNominal', 'var'), pNominal = vehicleParams(); end
if ~exist('vars', 'var'),     vars = ["speed", "fpa"]; end   % catalog Name(s)
if ~exist('N', 'var'),        N = 200; end
if ~exist('seed', 'var'),     seed = 1; end
if ~exist('runTag', 'var'),   runTag = ""; end               % added to the file name

catalog = mc_dispersion_catalog(pNominal);
% Ranges can be overridden per study, e.g.:
%   catalog.A(catalog.Name == "fpa") = 0;  catalog.B(catalog.Name == "fpa") = 60;
spec = mc_select(catalog, vars);
disp(spec);

%% ---- Run ----
samples = mc_generate_samples(spec, N, Seed=seed, Method="lhs");
fprintf('Running %d missions...\n', N);
results = mc_run(pNominal, samples, @mc_case_mission);

ud = results.Properties.UserData;
ud.spec = spec;
ud.seed = seed;
ud.nominalInputs = array2table(spec.Nominal', 'VariableNames', cellstr(spec.Name));
results.Properties.UserData = ud;

resultsDir = fullfile(projRoot, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
outFile = fullfile(resultsDir, "mc_mission_" + runTag + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + ".mat");
save(outFile, 'results');
fprintf('Saved %s (%.0f s total)\n', outFile, ud.totalTime);

%% ---- Analyse ----
outputs = ["feasible", "propBoostback", "propEntry", "landingMargin", "qPeak", ...
           "apogee", "ignitionSpeed", "ignitionMissX"];
if usejava('desktop')
    mc_analyze(results, vars, outputs, ColorBy="landingMargin", Title="Mission MC");
end

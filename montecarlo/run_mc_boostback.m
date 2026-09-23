%% RUN_MC_BOOSTBACK
% Monte Carlo / sensitivity study of the Phase 2 boostback guidance.
%
% Pipeline (each step is its own function, reusable for later phases):
%   mc_dispersion_catalog  what *can* be dispersed, with default ranges
%   mc_select              what *is* dispersed in this study
%   mc_generate_samples    N input sets (Latin hypercube, seeded)
%   mc_run                 one case per sample via a case function
%                          (here mc_case_boostback), errors captured
%   mc_analyze             statistics, sensitivity heatmap, plots
%
% Results are saved to results/ (not version-controlled) so a long run can
% be re-analysed without re-running it:
%   S = load('results/mc_boostback_<timestamp>.mat'); mc_analyze(S.results, ...)

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projRoot));

%% ---- Study configuration ----
pNominal = vehicleParams();
vars     = ["speed", "fpa"];   % any Name(s) from mc_dispersion_catalog
N        = 200;
seed     = 1;

catalog = mc_dispersion_catalog(pNominal);
% Ranges can be overridden per study, e.g.:
%   catalog.A(catalog.Name == "fpa") = 0;  catalog.B(catalog.Name == "fpa") = 60;
spec = mc_select(catalog, vars);
disp(spec);

%% ---- Run ----
samples = mc_generate_samples(spec, N, Seed=seed, Method="lhs");
fprintf('Running %d boostback cases...\n', N);
results = mc_run(pNominal, samples, @mc_case_boostback);

ud = results.Properties.UserData;
ud.spec = spec;
ud.seed = seed;
ud.nominalInputs = array2table(spec.Nominal', 'VariableNames', cellstr(spec.Name));
results.Properties.UserData = ud;

resultsDir = fullfile(projRoot, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
outFile = fullfile(resultsDir, "mc_boostback_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + ".mat");
save(outFile, 'results');
fprintf('Saved %s (%.0f s total)\n', outFile, ud.totalTime);

%% ---- Analyse ----
outputs = ["feasible", "propUsed", "propMargin", "burnTime", "T_coast", ...
           "apogee", "ignitionSpeed", "ignitionMissX"];
if usejava('desktop')
    mc_analyze(results, vars, outputs, ColorBy="propUsed", Title="Boostback MC");
end

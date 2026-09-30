function summary = mc_analyze(results, inputs, outputs, opts)
%MC_ANALYZE Summary statistics, sensitivities and plots for a Monte Carlo
%   results table from mc_run. Phase-agnostic: you name the input and
%   output columns to look at.
%
%   summary = mc_analyze(results, inputs, outputs)
%   summary = mc_analyze(..., ColorBy="propUsed", Title="...")
%
%   inputs  - dispersed variables (string array of results columns)
%   outputs - metric columns to summarize (numeric or logical)
%   ColorBy - output used to colour the input-space scatter
%   Title   - figure title prefix
%
%   Figures:
%     1. Input-space map of the first two inputs: feasible cases coloured
%        by ColorBy, infeasible cases as crosses, nominal as a star. (With
%        one input: ColorBy against that input.) This is the direct
%        answer to "where in the input space does it work, and at what
%        cost?", including interactions a single number can't show.
%     2. Sensitivity heatmap: Spearman rank correlation of every output
%        with every input. Rank correlation measures how *monotonically*
%        an output follows an input, without assuming linearity -- a
%        cheap, standard first screen. It's one input at a time and
%        misses non-monotonic effects; figure 1 is the check for those.
%        Metrics are computed on feasible cases, except "feasible"
%        itself, which uses all cases.
%     3. Histograms of the outputs over feasible cases.
%
%   summary - table of per-output statistics over feasible cases.

arguments
    results table
    inputs string
    outputs string
    opts.ColorBy (1,1) string = outputs(1)
    opts.Title (1,1) string = "Monte Carlo"
end

ok = results.status == "ok";
feas = ok;
if ismember("feasible", string(results.Properties.VariableNames))
    feas = ok & results.feasible;
end
nominal = results.Properties.UserData.nominal;

fprintf('\n%s: %d cases, %d feasible (%.1f%%), %d infeasible, %d errors\n', ...
    opts.Title, height(results), nnz(feas), 100*nnz(feas)/height(results), ...
    nnz(ok & ~feas), nnz(~ok));
if any(~ok)
    fprintf('  first error: %s\n', results.message(find(~ok, 1)));
end

% ---- Labels (names, units, display scaling) ----
lblIn  = arrayfun(@(n) mc_labels(n, results), inputs);
lblOut = arrayfun(@(n) mc_labels(n, results), outputs);
lblC   = mc_labels(opts.ColorBy, results);
refColor = [0.55 0.55 0.55];   % reference marks, visible in light and dark themes

% ---- Summary statistics over feasible cases (display units) ----
stats = zeros(numel(outputs), 6);
for j = 1:numel(outputs)
    y = double(results.(outputs(j))(feas)) * lblOut(j).scale;
    stats(j,:) = [mean(y), std(y), min(y), prctile_(y, 5), prctile_(y, 95), max(y)];
end
rowNames = cellstr(outputs + " [" + [lblOut.unit] + "]");
summary = array2table(stats, 'RowNames', rowNames, ...
    'VariableNames', {'Mean', 'Std', 'Min', 'P5', 'P95', 'Max'});
disp(summary);

% ---- 1. Input-space map ----
figure('Name', opts.Title + " - input space", 'Position', [80 80 820 600]);
c = double(results.(opts.ColorBy)) * lblC.scale;
if numel(inputs) >= 2
    x = results.(inputs(1)); y = results.(inputs(2));
    scatter(x(feas), y(feas), 36, c(feas), 'filled', 'DisplayName', 'Feasible case');
    hold on;
    plot(x(~feas), y(~feas), 'x', 'Color', [0.85 0.2 0.2], 'MarkerSize', 8, ...
         'LineWidth', 1.5, 'DisplayName', 'Infeasible / error');
    [xn, yn] = nominalInputs(results, inputs(1:2));
    plot(xn, yn, 'p', 'MarkerSize', 16, 'MarkerFaceColor', 'w', ...
         'MarkerEdgeColor', 'k', 'DisplayName', 'Nominal');
    ylabel(lblIn(2).text);
    cb = colorbar; cb.Label.String = lblC.text;
    legend('Location', 'bestoutside');
else
    x = results.(inputs(1));
    plot(x(feas), c(feas), 'o', 'DisplayName', 'Feasible case'); hold on;
    plot(x(~feas), zeros(nnz(~feas), 1), 'x', 'Color', [0.85 0.2 0.2], ...
         'DisplayName', 'Infeasible / error');
    ylabel(lblC.text); legend('Location', 'best');
end
xlabel(lblIn(1).text); grid on;
title(sprintf('%s: %d/%d feasible', opts.Title, nnz(feas), height(results)));

% ---- 2. Sensitivity heatmap (Spearman rank correlation) ----
rho = nan(numel(outputs), numel(inputs));
for j = 1:numel(outputs)
    rows = feas;
    if outputs(j) == "feasible", rows = ok; end
    for i = 1:numel(inputs)
        rho(j, i) = spearman(results.(inputs(i))(rows), double(results.(outputs(j))(rows)));
    end
end
figure('Name', opts.Title + " - sensitivity", ...
       'Position', [120 120 380 + 150*numel(inputs), 140 + 42*numel(outputs)]);
h = heatmap(cellstr([lblIn.short]), cellstr([lblOut.short]), round(rho, 2));
h.Colormap = redblue(); h.ColorLimits = [-1 1];
h.Title = "Rank correlation (output vs input)";
h.XLabel = "Dispersed input";
h.YLabel = "Output";

% ---- 3. Output histograms ----
nOut = numel(outputs);
figure('Name', opts.Title + " - outputs", 'Position', [160 160 1000 270*ceil(nOut/3)]);
tl = tiledlayout(ceil(nOut/3), min(nOut, 3), 'TileSpacing', 'compact');
title(tl, sprintf('%s: distribution over the %d feasible cases', opts.Title, nnz(feas)));
for j = 1:nOut
    ax = nexttile(tl);
    histogram(ax, double(results.(outputs(j))(feas)) * lblOut(j).scale);
    if isfield(nominal, outputs(j))
        xline(ax, double(nominal.(outputs(j))) * lblOut(j).scale, '--', 'Nominal', ...
              'Color', refColor, 'LineWidth', 1.2);
    end
    xlabel(ax, lblOut(j).text); ylabel(ax, 'Number of cases');
    grid(ax, 'on');
end
end

% =====================================================================
function [xn, yn] = nominalInputs(results, names)
% Nominal input values are stored by the entry script (UserData.nominalInputs).
ud = results.Properties.UserData;
xn = NaN; yn = NaN;
if isfield(ud, 'nominalInputs')
    xn = ud.nominalInputs.(names(1));
    yn = ud.nominalInputs.(names(2));
end
end

function r = spearman(x, y)
if numel(x) < 3 || all(y == y(1)) || all(x == x(1))
    r = NaN; return
end
C = corrcoef(tiedRank(x), tiedRank(y));
r = C(1, 2);
end

function r = tiedRank(x)
% Ranks with ties averaged (needed for logical outputs like "feasible").
[~, order] = sort(x);
r = zeros(size(x));
r(order) = 1:numel(x);
[~, ~, g] = unique(x);
meanRank = accumarray(g, r, [], @mean);
r = meanRank(g);
end

function q = prctile_(y, pct)
% Percentile by linear interpolation (avoids a toolbox dependency).
y = sort(y(:));
if isempty(y), q = NaN; return; end
pos = 1 + (numel(y) - 1) * pct / 100;
q = interp1(1:numel(y), y, pos);
if numel(y) == 1, q = y; end
end

function cmap = redblue()
n = 128;
up = linspace(0, 1, n)';
cmap = [[up; ones(n,1)], [up; flipud(up)], [ones(n,1); flipud(up)]];
end

function fig = plot_flight_record(src, opts)
%PLOT_FLIGHT_RECORD Plot a saved flight record. Runs no simulation.
%
%   plot_flight_record()                      % most recent record
%   plot_flight_record("nominal")             % most recent "nominal*" record
%   plot_flight_record(file)                  % a specific .mat file
%   plot_flight_record(rec)                   % a record already in memory
%   plot_flight_record(..., Panels=["thrust" "dynPressure" "mach"])
%   plot_flight_record(..., Trajectory=false)
%   plot_flight_record(..., Title="Nominal return")  % custom figure title
%
%   Layout: the trajectory (downrange vs altitude, coloured by flight
%   phase, events marked) across the top, then one panel per signal name in
%   Panels, against time. Each panel shows the signal's limits from the
%   record (e.g. thrust min/max, dynamic-pressure limit) and the event
%   times. Any signal stored in the record can be named in Panels -- see
%   flight_signal_defs for the catalog, or rec.signals for what a file has.

arguments
    src = []
    opts.Panels (1,:) string = ["thrust", "loadFactor", "thrustAngle", ...
                                "speed", "dynPressure", "heatIndex"]
    opts.Trajectory (1,1) logical = true
    opts.Title (1,1) string = ""          % default: record name, source, date, code version
end

if isstruct(src)
    rec = src;
elseif isempty(src)
    rec = load_flight_record();
else
    rec = load_flight_record(src);
end

refColor = [0.55 0.55 0.55];
phaseColors = containers.Map({'boostback', 'coast', 'entry burn', 'landing burn'}, ...
    {[0.00 0.45 0.74], [0.85 0.33 0.10], [0.93 0.69 0.13], [0.47 0.67 0.19]});
sig = rec.signals;
t = sig.t;
nPanels = numel(opts.Panels);
nRows = ceil(nPanels / 2) + opts.Trajectory;

fig = figure('Name', "Flight record: " + rec.meta.name, 'Position', [60 40 1150 240 * nRows + 60]);
tl = tiledlayout(nRows, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
if strlength(opts.Title) > 0
    title(tl, opts.Title, 'Interpreter', 'none');
else
    title(tl, sprintf('%s  (%s, %s, code %s)', rec.meta.name, rec.meta.source, ...
          string(rec.meta.created, 'yyyy-MM-dd HH:mm'), rec.meta.gitCommit), 'Interpreter', 'none');
end

% ---- Trajectory ----
if opts.Trajectory
    ax = nexttile(tl, [1 2]); hold(ax, 'on');
    xd = displayOf(rec, "x"); yd = displayOf(rec, "altitude");
    for ph = categories(sig.phase)'
        sel = sig.phase == ph{1};
        if ~any(sel), continue, end
        x = sig.x * xd.displayScale; y = sig.altitude * yd.displayScale;
        x(~sel) = NaN; y(~sel) = NaN;            % gaps outside this phase
        plot(ax, x, y, 'LineWidth', 1.8, 'Color', phaseColors(ph{1}), 'DisplayName', ph{1});
    end
    plot(ax, 0, 0, '^', 'MarkerSize', 9, 'Color', refColor, 'MarkerFaceColor', refColor, ...
         'DisplayName', 'Launch site');
    if isfield(rec.params, 'target')
        plot(ax, rec.params.target.x * xd.displayScale, 0, 's', 'MarkerSize', 9, ...
             'Color', [0.85 0.2 0.2], 'MarkerFaceColor', [0.85 0.2 0.2], 'DisplayName', 'Landing site');
    end
    for k = 2:height(rec.events)                 % skip "Separation" (start of the line)
        i = find(t >= rec.events.t(k), 1);
        if isempty(i), i = numel(t); end
        plot(ax, sig.x(i) * xd.displayScale, sig.altitude(i) * yd.displayScale, 'o', ...
             'MarkerSize', 5, 'Color', refColor, 'HandleVisibility', 'off');
        text(ax, sig.x(i) * xd.displayScale, sig.altitude(i) * yd.displayScale, ...
             "  " + rec.events.name(k), 'FontSize', 8, 'Color', refColor);
    end
    grid(ax, 'on');
    xlabel(ax, sprintf('%s [%s]', xd.description, xd.displayUnit));
    ylabel(ax, sprintf('%s [%s]', yd.description, yd.displayUnit));
    title(ax, 'Trajectory');
    legend(ax, 'Location', 'bestoutside');
end

% ---- Time histories ----
for k = 1:nPanels
    name = opts.Panels(k);
    if ~ismember(name, string(sig.Properties.VariableNames))
        error('plot_flight_record:unknownSignal', ...
            'Signal "%s" is not in this record. Available: %s', name, ...
            strjoin(setdiff(string(sig.Properties.VariableNames), ["t" "phase"]), ', '));
    end
    d = displayOf(rec, name);
    ax = nexttile(tl); hold(ax, 'on');
    colorOrder = get(groot, 'defaultAxesColorOrder');
    drawSeries(ax, t, sig.(name) * d.displayScale, d.plotStyle, '-', 1.6, ...
               colorOrder(1,:), d.description);
    note = "";
    if isfield(rec.limits, name)
        lim = rec.limits.(name);
        anyLimit = false;
        if ~isempty(lim.upper) && any(isfinite(lim.upper))
            drawSeries(ax, t, lim.upper * d.displayScale, d.plotStyle, '--', 1.1, refColor, lim.upperLabel);
            anyLimit = true;
        end
        if ~isempty(lim.lower) && any(isfinite(lim.lower))
            drawSeries(ax, t, lim.lower * d.displayScale, d.plotStyle, ':', 1.3, refColor, lim.lowerLabel);
            anyLimit = true;
        end
        if ~anyLimit, note = " (no limit active)"; end
    end
    for e = 2:height(rec.events)
        xline(ax, rec.events.t(e), ':', 'Color', refColor, 'HandleVisibility', 'off');
    end
    grid(ax, 'on');
    unitText = "";
    if strlength(d.displayUnit) > 0, unitText = " [" + d.displayUnit + "]"; end
    ylabel(ax, d.description + unitText);
    title(ax, d.description + note);
    if k > nPanels - 2, xlabel(ax, 'Time from separation [s]'); end
    if numel(ax.Children) > 1, legend(ax, 'Location', 'best'); end
end
end

% =====================================================================
function d = displayOf(rec, name)
k = find([rec.display.name] == name, 1);
if isempty(k)
    d = struct('name', name, 'description', name, 'displayUnit', "", ...
               'displayScale', 1, 'plotStyle', "line");
else
    d = rec.display(k);
end
end

function drawSeries(ax, t, y, style, lineSpec, width, color, label)
if style == "stairs"
    stairs(ax, t, y, lineSpec, 'LineWidth', width, 'Color', color, 'DisplayName', label);
else
    plot(ax, t, y, lineSpec, 'LineWidth', width, 'Color', color, 'DisplayName', label);
end
end

function T = study_entry_burn(p, opts)
%STUDY_ENTRY_BURN Trade study: entry-burn cost versus the descent limits.
%
%   T = study_entry_burn()                 % nominal vehicleParams()
%   T = study_entry_burn(p, Plot=false)
%
%   The boostback is solved once; every point then flies boostback + coast
%   + sized entry burn + coast through the COESA plant
%   (simulate_boostback_coast), with the burn sized online by
%   entry_burn_sizing from the plant state at the start altitude.
%
%   Three sweeps (each varies one thing, other limits inactive):
%     "qMax"    - peak dynamic pressure limit
%     "heatRel" - peak heating index sqrt(rho)*V^3, relative to no burn
%     "startAlt"- burn start altitude, at a fixed qMax (opts.QMaxForAltScan):
%                 how much propellant the fixed-altitude choice costs vs
%                 starting later
%
%   Returns a table, one row per point: the swept value, burn delta-v and
%   propellant, feasibility, plant peak q / heating / heat load (relative
%   to no burn), the guidance model's predicted peak q (model error), speed
%   at ignition, and the landing margin: propellant left above dry mass
%   minus the ideal (impulsive, gravity-free) landing-burn propellant for
%   the ignition speed.

arguments
    p struct = vehicleParams()
    opts.Plot (1,1) logical = true
    opts.QMax (1,:) double = (50:10:230) * 1e3
    opts.HeatRel (1,:) double = 0.3:0.05:1
    opts.StartAlt (1,:) double = (30:5:80) * 1e3
    opts.QMaxForAltScan (1,1) double = 150e3
end

warnState = warning('off', 'aeroblks:aeroblkcoesa:AltMsg');
restoreWarn = onCleanup(@() warning(warnState));

state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];
bb = boostback_guidance(state0, p);

base = p; base.entry.enabled = false;
ref = flyAndMeasure(state0, bb, base, [], []);

inactive = p;
inactive.entry.enabled = true;
inactive.entry.qMax = Inf; inactive.entry.heatMaxRel = Inf; inactive.entry.vIgnitionMax = Inf;

rows = {};
for q = opts.QMax
    pp = inactive; pp.entry.qMax = q;
    rows{end+1} = flyAndMeasure(state0, bb, pp, "qMax", q, ref); %#ok<AGROW>
end
for h = opts.HeatRel
    pp = inactive; pp.entry.heatMaxRel = h;
    rows{end+1} = flyAndMeasure(state0, bb, pp, "heatRel", h, ref); %#ok<AGROW>
end
for a = opts.StartAlt
    pp = inactive; pp.entry.qMax = opts.QMaxForAltScan; pp.entry.startAltitude = a;
    rows{end+1} = flyAndMeasure(state0, bb, pp, "startAlt", a, ref); %#ok<AGROW>
end
T = struct2table([rows{:}]);
T.Properties.UserData = struct('noBurn', ref, 'target_x', p.target.x, ...
    'boostbackPropLeft', bb.cutoffState(5) - p.dryMass);

fprintf('\n--- Entry burn trade study (landing site x = %.0f km, start %.0f km, %d engine(s)) ---\n', ...
        p.target.x/1e3, p.entry.startAltitude/1e3, p.nEnginesEntry);
fprintf('No entry burn: peak q %.0f kPa, V_ign %.0f m/s, landing margin %.0f kg\n', ...
        ref.qPeak/1e3, ref.vIgnition, ref.landingMargin);
fprintf('Smallest reachable peak q: %.0f kPa (all propellant above the %.0f kg landing reserve)\n', ...
        min(T.qPeak(T.sweep == "qMax"))/1e3, p.landingPropReserve);

if opts.Plot
    plotTrade(T, ref, p, opts);
end
end

% =====================================================================
function r = flyAndMeasure(state0, bb, p, sweep, value, ref)
tr = simulate_boostback_coast(state0, bb, p);
X = tr.XB;
q = zeros(size(X, 1), 1); heat = q;
for i = 1:size(X, 1)
    [~, in] = dragAcceleration(X(i,:)', p);
    q(i) = in.q; heat(i) = in.heat;
end
ign = tr.ignition;
c = p.Isp * p.g0;
r.sweep     = string(sweep);
r.value     = double(value);
r.dv        = 0; r.propEntry = 0; r.feasible = true; r.qPredicted = NaN;
if ~isempty(tr.entry)
    r.dv = tr.entry.dv; r.propEntry = tr.entry.propUsed;
    r.feasible = tr.entry.feasible; r.qPredicted = tr.entry.predicted.qPeak;
end
r.qPeak     = max(q);
r.heatPeak  = max(heat);
r.heatLoad  = trapz(tr.tB, heat);
r.vIgnition = norm(ign(3:4));
r.xIgnition = ign(1);
r.landingMargin = (ign(5) - p.dryMass) - ign(5) * (1 - exp(-r.vIgnition / c));
if nargin >= 6 && ~isempty(ref)
    r.heatPeakRel = r.heatPeak / ref.heatPeak;
    r.heatLoadRel = r.heatLoad / ref.heatLoad;
else
    r.heatPeakRel = 1; r.heatLoadRel = 1;
end
end

function plotTrade(T, ref, p, opts)
figure('Name', 'Entry burn trade study', 'Position', [80 60 1150 820]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Entry burn trade study (landing site x = %.0f km, start %.0f km)', ...
      p.target.x/1e3, p.entry.startAltitude/1e3));

Q = T(T.sweep == "qMax", :);
ok = Q.feasible;
ax = nexttile(tl);
plot(ax, Q.value(ok)/1e3, Q.propEntry(ok), 'o-', 'LineWidth', 1.5, 'DisplayName', 'Entry-burn propellant');
hold(ax, 'on');
plot(ax, Q.value(ok)/1e3, Q.landingMargin(ok), 's-', 'LineWidth', 1.5, 'DisplayName', 'Landing margin');
yline(ax, 0, 'k:', 'HandleVisibility', 'off');
if any(~ok)
    xline(ax, max(Q.value(~ok))/1e3, '--', 'Not reachable below', 'Color', [0.85 0.2 0.2], ...
          'HandleVisibility', 'off');
end
grid(ax, 'on'); xlabel(ax, 'Peak dynamic pressure limit [kPa]'); ylabel(ax, 'Propellant [kg]');
title(ax, 'Cost of a dynamic-pressure limit'); legend(ax, 'Location', 'best');

ax = nexttile(tl);
plot(ax, Q.value/1e3, Q.qPeak/1e3, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Plant (COESA)'); hold(ax, 'on');
plot(ax, Q.value/1e3, Q.qPredicted/1e3, 'x--', 'LineWidth', 1.2, 'DisplayName', 'Guidance prediction');
plot(ax, Q.value/1e3, Q.value/1e3, 'k:', 'DisplayName', 'Limit');
grid(ax, 'on'); xlabel(ax, 'Peak dynamic pressure limit [kPa]'); ylabel(ax, 'Peak q achieved [kPa]');
title(ax, 'Limit vs achieved (model error)'); legend(ax, 'Location', 'best');

H = T(T.sweep == "heatRel", :);
ax = nexttile(tl);
plot(ax, H.value, H.propEntry, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Entry-burn propellant'); hold(ax, 'on');
plot(ax, H.value, H.landingMargin, 's-', 'LineWidth', 1.5, 'DisplayName', 'Landing margin');
yyaxis(ax, 'right');
plot(ax, H.value, H.qPeak/1e3, 'd-', 'LineWidth', 1.2, 'DisplayName', 'Peak q [kPa]');
ylabel(ax, 'Peak q [kPa]');
yyaxis(ax, 'left'); ylabel(ax, 'Propellant [kg]');
yline(ax, 0, 'k:', 'HandleVisibility', 'off');
grid(ax, 'on'); xlabel(ax, 'Peak heating limit (relative to no burn)');
title(ax, 'Cost of a heating limit'); legend(ax, 'Location', 'best');

A = T(T.sweep == "startAlt", :);
ax = nexttile(tl);
okA = A.feasible;
plot(ax, A.value(okA)/1e3, A.propEntry(okA), 'o-', 'LineWidth', 1.5); hold(ax, 'on');
plot(ax, A.value(~okA)/1e3, A.propEntry(~okA), 'x', 'Color', [0.85 0.2 0.2], 'MarkerSize', 9, 'LineWidth', 1.5);
xline(ax, p.entry.startAltitude/1e3, '--', 'Baseline', 'HandleVisibility', 'off');
grid(ax, 'on'); xlabel(ax, 'Entry burn start altitude [km]'); ylabel(ax, 'Entry-burn propellant [kg]');
title(ax, sprintf('Start altitude at q_{max} = %.0f kPa (x = limit not met)', opts.QMaxForAltScan/1e3));
end

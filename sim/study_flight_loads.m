function S = study_flight_loads(p, opts)
%STUDY_FLIGHT_LOADS Flight loads along one mission (Simulink model), with
%   and without the entry burn, from separation to landing-burn ignition.
%
%   S = study_flight_loads()            % nominal vehicleParams()
%   S = study_flight_loads(p, Plot=false)
%
%   Flies the mission through simulate_mission twice (entry burn on / off)
%   and compares dynamic pressure, axial (sensed) acceleration and the
%   Sutton-Graves heat-flux estimate (see flight_loads for definitions and
%   assumptions). Returns the two flight_loads structs (S.withEntry,
%   S.noEntry) and prints the peaks.

arguments
    p struct = vehicleParams()
    opts.Plot (1,1) logical = true
end

pOn = p;  pOn.entry.enabled = true;
pOff = p; pOff.entry.enabled = false;
fOn = simulate_mission(pOn);
fOff = simulate_mission(pOff);
S.withEntry = flight_loads(fOn, pOn);
S.noEntry = flight_loads(fOff, pOff);
S.flightWithEntry = fOn;

fprintf('\n--- Flight loads, separation -> landing-burn ignition (landing site x = %.0f km) ---\n', p.target.x/1e3);
fprintf('%-34s %12s %12s\n', '', 'no entry', 'entry burn');
row = @(name, a, b, fmt) fprintf(['%-34s ' fmt ' ' fmt '\n'], name, a, b);
row('Max-Q [kPa]',                  S.noEntry.maxQ/1e3,          S.withEntry.maxQ/1e3,          '%12.1f');
row('  at altitude [km]',           S.noEntry.maxQAltitude/1e3,  S.withEntry.maxQAltitude/1e3,  '%12.1f');
row('  at Mach',                    S.noEntry.maxQMach,          S.withEntry.maxQMach,          '%12.2f');
row('Peak axial accel, boostback [g]', S.noEntry.nAxialMaxBoostback, S.withEntry.nAxialMaxBoostback, '%12.2f');
row('Peak axial accel, descent [g]',   S.noEntry.nAxialMaxDescent,   S.withEntry.nAxialMaxDescent,   '%12.2f');
row('Peak heat flux [kW/m^2] (est.)', S.noEntry.heatFluxMax/1e3, S.withEntry.heatFluxMax/1e3, '%12.0f');
row('Heat load [MJ/m^2] (est.)',     S.noEntry.heatLoad/1e6,     S.withEntry.heatLoad/1e6,     '%12.2f');
fprintf('(heat flux: Sutton-Graves with assumed effective nose radius %.2f m -- order of magnitude)\n', ...
        p.aero.effectiveNoseRadius);

if ~opts.Plot, return; end
figure('Name', 'Flight loads', 'Position', [100 60 1000 820]);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Flight loads from separation to landing-burn ignition (landing site x = %.0f km)', p.target.x/1e3));
series = {'q', 1e-3, 'Dynamic pressure [kPa]';
          'nAxial', 1, 'Axial (sensed) acceleration [g]';
          'heatFlux', 1e-3, 'Stagnation heat flux [kW/m^2] (Sutton-Graves est.)'};
for i = 1:3
    ax = nexttile(tl);
    plot(ax, S.noEntry.t, S.noEntry.(series{i,1}) * series{i,2}, '--', 'LineWidth', 1.4, ...
         'DisplayName', 'No entry burn');
    hold(ax, 'on');
    plot(ax, S.withEntry.t, S.withEntry.(series{i,1}) * series{i,2}, 'LineWidth', 1.8, ...
         'DisplayName', 'With entry burn');
    if i == 1 && isfinite(p.entry.qMax)
        yline(ax, p.entry.qMax / 1e3, ':', 'q limit', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
    end
    ev = fOn.ev;
    xline(ax, ev.tCutoff, ':', 'Boostback cutoff', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
    if ~isempty(ev.tEntry)
        xline(ax, ev.tEntry(1), ':', 'Entry burn', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
    end
    grid(ax, 'on'); ylabel(ax, series{i,3}); legend(ax, 'Location', 'northwest');
    if i == 3, xlabel(ax, 'Time from separation [s]'); end
end
end

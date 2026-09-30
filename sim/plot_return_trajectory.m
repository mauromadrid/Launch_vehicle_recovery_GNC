function fig = plot_return_trajectory(t, X, T, theta, p, ev, figName)
%PLOT_RETURN_TRAJECTORY Standard results figure for a booster-return run,
%   shared by the MATLAB demo and the Simulink runner so both are
%   compared like for like.
%
%   t      - n x 1 time [s]
%   X      - n x 5 state history [x y vx vy m]
%   T      - n x 1 thrust magnitude [N]
%   theta  - n x 1 thrust angle from +x axis [rad] (ignored where T = 0)
%   p      - parameter struct from vehicleParams()
%   ev     - event times [s]: ev.tCutoff (boostback cutoff),
%            ev.tIgnite (landing-burn ignition); NaN if not reached;
%            optional ev.tEntry = [start end] of an entry burn
%   figName- figure title
%
%   Note the flight-path angle (direction of the velocity) and the thrust
%   angle (direction the engine pushes) are different things: during the
%   boostback the stage is still flying downrange and up while thrusting
%   back toward the landing site, so they differ by ~150 deg.

isBurning = T > 0;
refColor = [0.55 0.55 0.55]; % reference lines/markers, visible in light and dark themes
phaseColor = lines(3);
phaseIdx = ones(size(t));                        % 1 = boostback
phaseIdx(t >= ev.tCutoff) = 2;                   % 2 = coast
phaseIdx(t >= ev.tIgnite) = 3;                   % 3 = landing burn
phaseNames = {'Boostback', 'Coast', 'Landing burn'};

fig = figure('Name', figName, 'Position', [100 50 1100 1000]);
tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, figName);

% ---- Trajectory, coloured by phase ----
ax = nexttile(tl, [1 2]);
hold(ax, 'on');
for k = 1:3
    sel = phaseIdx == k;
    if any(sel)
        plot(ax, X(sel,1)/1e3, X(sel,2)/1e3, 'LineWidth', 1.8, ...
             'Color', phaseColor(k,:), 'DisplayName', phaseNames{k});
    end
end
plot(ax, 0, 0, '^', 'MarkerSize', 9, 'Color', refColor, ...
     'MarkerFaceColor', refColor, 'DisplayName', 'Launch site');
plot(ax, p.target.x/1e3, p.target.y/1e3, 'rs', 'MarkerSize', 9, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Landing site');
plot(ax, p.target.x/1e3, p.landingIgniteAltitude/1e3, 'ro', ...
     'DisplayName', 'Aimed ignition point');
grid(ax, 'on');
xAll = [X(:,1); 0; p.target.x] / 1e3;
xlim(ax, [min(xAll) max(xAll)] + [-0.05 0.05] * max(range(xAll), 1));
xlabel(ax, 'Downrange [km]'); ylabel(ax, 'Altitude [km]');
title(ax, 'Trajectory');
legend(ax, 'Location', 'bestoutside');

% ---- Time histories ----
ax = nexttile(tl);
plot(ax, t, X(:,2)/1e3, 'LineWidth', 1.5);
decorate(ax, ev, 'Altitude [km]', 'Altitude');

ax = nexttile(tl);
plot(ax, t, vecnorm(X(:,3:4), 2, 2), 'LineWidth', 1.5);
decorate(ax, ev, 'Speed [m/s]', 'Speed');

ax = nexttile(tl);
stairs(ax, t, T/1e3, 'LineWidth', 1.5, 'DisplayName', 'Thrust');
hold(ax, 'on');
yline(ax, p.T_max_boostback/1e3, '--', 'Color', refColor, 'DisplayName', 'Boostback T_{max}');
yline(ax, p.T_min_boostback/1e3, ':',  'Color', refColor, 'DisplayName', 'Boostback T_{min}');
legend(ax, 'Location', 'best');
decorate(ax, ev, 'Thrust [kN]', 'Thrust magnitude');

ax = nexttile(tl);
gammaDeg = rad2deg(unwrap(atan2(X(:,4), X(:,3))));
thetaDeg = nan(size(t));
thetaDeg(isBurning) = rad2deg(unwrap(theta(isBurning)));
plot(ax, t, gammaDeg, 'LineWidth', 1.5, 'DisplayName', 'Flight-path angle \gamma');
hold(ax, 'on');
plot(ax, t, thetaDeg, 'LineWidth', 1.5, 'DisplayName', 'Thrust angle \theta');
legend(ax, 'Location', 'best');
decorate(ax, ev, 'Angle from +x (downrange) [deg]', 'Flight-path vs thrust angle');

ax = nexttile(tl);
plot(ax, t, X(:,5), 'LineWidth', 1.5);
hold(ax, 'on');
yline(ax, p.dryMass, '--', 'Dry mass', 'Color', refColor);
decorate(ax, ev, 'Mass [kg]', 'Mass');

ax = nexttile(tl);
stairs(ax, t, T ./ X(:,5) / p.g0, 'LineWidth', 1.5);
decorate(ax, ev, 'Thrust accel. [g]', 'Thrust acceleration (T/m)');
end

function decorate(ax, ev, yLabel, titleStr)
grid(ax, 'on');
xlabel(ax, 'Time [s]'); ylabel(ax, yLabel); title(ax, titleStr);
if isfinite(ev.tCutoff), xline(ax, ev.tCutoff, ':', 'Cutoff', 'HandleVisibility', 'off'); end
if isfinite(ev.tIgnite), xline(ax, ev.tIgnite, ':', 'Ignition', 'HandleVisibility', 'off'); end
if isfield(ev, 'tEntry') && ~isempty(ev.tEntry)
    xline(ax, ev.tEntry(1), ':', 'Entry burn', 'HandleVisibility', 'off');
end
end

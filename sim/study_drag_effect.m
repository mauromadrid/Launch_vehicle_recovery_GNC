function out = study_drag_effect(p, opts)
%STUDY_DRAG_EFFECT What drag does to a boostback planned in vacuum.
%
%   out = study_drag_effect()          % nominal vehicleParams()
%   out = study_drag_effect(p)         % any parameter set
%   out = study_drag_effect(p, Plot=false)
%
%   Solves the (vacuum) boostback guidance once, then flies the same plan
%   through a vacuum plant and through the drag plant (COESA + CD(Mach)),
%   each down to landing-burn ignition altitude, and compares:
%     - downrange error at ignition altitude, drag minus vacuum (what a
%       bias must correct)
%     - speed / Mach at ignition, and the propellant an ideal landing burn
%       would need to cancel that speed (with vs without drag)
%     - peak dynamic pressure and peak drag deceleration (loads; what an
%       entry burn would be sized to limit)
%   The guidance is not changed: this measures the problem, it doesn't
%   solve it.

arguments
    p struct = vehicleParams()
    opts.Plot (1,1) logical = true
end

% Drag only: no entry burn in either case (study_entry_burn covers that)
p.entry.enabled = false;
pVac = p;  pVac.aero.dragEnabled = false;
pDrag = p; pDrag.aero.dragEnabled = true;

state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];
bb = boostback_guidance(state0, p);
vac  = simulate_boostback_coast(state0, bb, pVac);
drag = simulate_boostback_coast(state0, bb, pDrag);

% Drag diagnostics along the drag trajectory (burn + coast)
tD = [drag.tA; drag.tA(end) + drag.tB];
XD = [drag.XA; drag.XB];
info = arrayfun(@(i) diagOf(XD(i,:)', pDrag), (1:numel(tD))');
q     = [info.q]';
decel = [info.decel]';
mach  = [info.mach]';
[qMax, iq] = max(q);
[dMax, id] = max(decel);

ignVac  = vac.ignition;
ignDrag = drag.ignition;

% Ideal (impulsive, gravity-free) landing-burn propellant to cancel the
% arrival speed -- a lower bound, to show the order of magnitude
c = p.Isp * p.g0;
propFor = @(ign) ign(5) * (1 - exp(-norm(ign(3:4)) / c));

out = struct( ...
    'target_x',            p.target.x, ...
    'burnTime',            bb.duration, ...
    'propBoostback',       bb.propUsed, ...
    'propLeftAtCutoff',    bb.cutoffState(5) - p.dryMass, ...
    'downrangeError',      ignDrag(1) - ignVac(1), ...
    'coastTimeVac',        vac.tB(end), ...
    'coastTimeDrag',       drag.tB(end), ...
    'ignitionSpeedVac',    norm(ignVac(3:4)), ...
    'ignitionSpeedDrag',   norm(ignDrag(3:4)), ...
    'ignitionMachDrag',    mach(end), ...
    'ignitionFPADrag',     atan2d(ignDrag(4), ignDrag(3)), ...
    'landingPropIdealVac', propFor(ignVac), ...
    'landingPropIdealDrag', propFor(ignDrag), ...
    'maxQ',                qMax, ...
    'maxQAltitude',        XD(iq, 2), ...
    'maxQMach',            mach(iq), ...
    'maxDragDecelG',       dMax / p.g0, ...
    'maxDecelAltitude',    XD(id, 2), ...
    'dragDeltaV',          trapz(tD, decel));

fprintf('\n--- Drag effect on a vacuum-planned boostback (landing site x = %.0f km) ---\n', p.target.x/1e3);
fprintf('Boostback: %.1f s, %.0f kg used, %.0f kg left above dry mass\n', ...
        out.burnTime, out.propBoostback, out.propLeftAtCutoff);
fprintf('At ignition altitude (%.0f m):      vacuum      drag\n', p.landingIgniteAltitude);
fprintf('  downrange [km]                 %8.2f  %8.2f   (drag - vacuum: %+.2f km)\n', ...
        ignVac(1)/1e3, ignDrag(1)/1e3, out.downrangeError/1e3);
fprintf('  coast time [s]                 %8.1f  %8.1f\n', out.coastTimeVac, out.coastTimeDrag);
fprintf('  speed [m/s]                    %8.0f  %8.0f   (Mach %.2f)\n', ...
        out.ignitionSpeedVac, out.ignitionSpeedDrag, out.ignitionMachDrag);
fprintf('  ideal landing prop [kg]        %8.0f  %8.0f   (have %.0f)\n', ...
        out.landingPropIdealVac, out.landingPropIdealDrag, out.propLeftAtCutoff);
fprintf('Speed removed by drag: %.0f m/s\n', out.dragDeltaV);
fprintf('Peak dynamic pressure: %.1f kPa at %.1f km (Mach %.1f)\n', ...
        qMax/1e3, out.maxQAltitude/1e3, out.maxQMach);
fprintf('Peak drag deceleration: %.1f g at %.1f km\n', out.maxDragDecelG, out.maxDecelAltitude/1e3);

if ~opts.Plot, return; end

XV = [vac.XA; vac.XB];
speedV = vecnorm(XV(:,3:4), 2, 2);
speedD = vecnorm(XD(:,3:4), 2, 2);

figure('Name', 'Drag effect', 'Position', [100 60 1100 820]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Vacuum-planned boostback flown with and without drag (landing site x = %.0f km)', p.target.x/1e3));

ax = nexttile(tl, [1 2]);
plot(ax, XV(:,1)/1e3, XV(:,2)/1e3, '--', 'LineWidth', 1.5, 'DisplayName', 'Vacuum plant'); hold(ax, 'on');
plot(ax, XD(:,1)/1e3, XD(:,2)/1e3, 'LineWidth', 1.8, 'DisplayName', 'Drag plant (COESA)');
plot(ax, p.target.x/1e3, p.landingIgniteAltitude/1e3, 'ro', 'DisplayName', 'Aimed ignition point');
grid(ax, 'on'); xlabel(ax, 'Downrange [km]'); ylabel(ax, 'Altitude [km]');
title(ax, 'Trajectory'); legend(ax, 'Location', 'bestoutside');

ax = nexttile(tl);
plot(ax, speedV, XV(:,2)/1e3, '--', 'LineWidth', 1.5); hold(ax, 'on');
plot(ax, speedD, XD(:,2)/1e3, 'LineWidth', 1.8);
grid(ax, 'on'); xlabel(ax, 'Speed [m/s]'); ylabel(ax, 'Altitude [km]');
title(ax, 'Altitude vs speed'); legend(ax, {'Vacuum', 'Drag'}, 'Location', 'best');

ax = nexttile(tl);
plot(ax, q/1e3, XD(:,2)/1e3, 'LineWidth', 1.8);
grid(ax, 'on'); xlabel(ax, 'Dynamic pressure [kPa]'); ylabel(ax, 'Altitude [km]');
title(ax, 'Dynamic pressure (drag plant)');

ax = nexttile(tl);
plot(ax, tD, decel / p.g0, 'LineWidth', 1.8);
grid(ax, 'on'); xlabel(ax, 'Time [s]'); ylabel(ax, 'Drag deceleration [g]');
title(ax, 'Drag deceleration');

ax = nexttile(tl);
plot(ax, tD, mach, 'LineWidth', 1.8);
grid(ax, 'on'); xlabel(ax, 'Time [s]'); ylabel(ax, 'Mach');
title(ax, 'Mach number (drag plant)');
end

function s = diagOf(state, p)
[~, s] = dragAcceleration(state, p);
end

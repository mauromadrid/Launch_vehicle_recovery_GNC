function flight = simulate_mission(p, opts)
%SIMULATE_MISSION Run the main simulation -- the Simulink model
%   booster_return_3dof -- for a given parameter set, and return the flight.
%
%   flight = simulate_mission()          % nominal vehicleParams()
%   flight = simulate_mission(p)         % any (e.g. dispersed) parameters
%
%   This is the single entry point for "fly the mission": run_and_plot and
%   the Monte Carlo case both call it. It computes the pre-flight guidance
%   with mission_setup(p), then runs the model through a
%   Simulink.SimulationInput that supplies p, bb, ld as variables and
%   disables the model's InitFcn -- which would otherwise rebuild them from
%   vehicleParams() and silently discard the parameters passed in.
%
%   flight fields:
%     t, X        - time [s], states [x y vx vy m] (logged)
%     T, theta    - thrust command [N], thrust angle [rad]
%     mCutoff, entrySized - entry-burn cutoff mass and "sized" flag, as the
%                   model's MATLAB Function block computed them in flight
%     bb, ld, p   - the guidance plans and parameters actually flown
%     ev          - event times: tCutoff (boostback), tEntry ([start end],
%                   [] if none), tIgnite (landing-burn ignition)
%     ignition    - state at landing-burn ignition altitude (interpolated
%                   to the exact crossing)
%     simTime     - wall-clock time of the Simulink run [s]

arguments
    p struct = vehicleParams()
    opts.Model (1,1) string = "booster_return_3dof"
end

mdl = char(opts.Model);
projRoot = fileparts(fileparts(mfilename('fullpath')));
if ~bdIsLoaded(mdl)
    load_system(fullfile(projRoot, 'simulink', [mdl '.slx']));
end

[bb, ld] = mission_setup(p);

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('InitFcn', '');   % p, bb, ld come from here instead
in = in.setVariable('p', p);
in = in.setVariable('bb', bb);
in = in.setVariable('ld', ld);

% The COESA block warns once per run that it extrapolates above 84.85 km;
% expected, since its altitude input is clamped at p.atmo.maxAltitude.
wState = warning('off', 'aeroblks:aeroblkcoesa:AltMsg');
restoreWarn = onCleanup(@() warning(wState));
tSim = tic;
out = sim(in, 'ShowProgress', 'off');
simTime = toc(tSim);
if ~isempty(out.ErrorMessage)
    error('simulate_mission:simFailed', 'Simulink run failed: %s', out.ErrorMessage);
end

log = out.get('simlog');
t = log.time;
D = log.signals.values;   % columns: x y vx vy m T_cmd theta_cmd mCutoff entrySized

flight.t = t;
flight.X = D(:, 1:5);
flight.T = D(:, 6);
flight.theta = D(:, 7);
flight.mCutoff = D(end, 8);
flight.entrySized = D(end, 9) > 0.5;
flight.bb = bb; flight.ld = ld; flight.p = p;
flight.simTime = simTime;

% Events
ev.tCutoff = bb.duration;
iIgn = find(t > bb.duration & D(:,2) <= p.landingIgniteAltitude, 1);
ev.tIgnite = NaN;
flight.ignition = nan(5, 1);
if ~isempty(iIgn)
    % interpolate between the samples bracketing the crossing
    f = (D(iIgn-1, 2) - p.landingIgniteAltitude) / (D(iIgn-1, 2) - D(iIgn, 2));
    flight.ignition = (D(iIgn-1, 1:5) + f * (D(iIgn, 1:5) - D(iIgn-1, 1:5)))';
    ev.tIgnite = t(iIgn-1) + f * (t(iIgn) - t(iIgn-1));
end
entryOn = t > bb.duration & t < ev.tIgnite & D(:, 6) > 0;
ev.tEntry = [];
if any(entryOn)
    ev.tEntry = [t(find(entryOn, 1)) t(find(entryOn, 1, 'last'))];
end
flight.ev = ev;
end

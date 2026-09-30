function m = mc_case_mission(p)
%MC_CASE_MISSION Monte Carlo case: fly the main simulation (the Simulink
%   model, via simulate_mission) with parameter set p and reduce the
%   flight to scalar metrics.
%
%   All mission logic lives in simulate_mission / the model; this function
%   only measures. A boostback that has no feasible solution is an outcome
%   (feasible = false, no flight), not an error.
%
%   Metrics:
%     feasible        boostback guidance found a solution
%     burnTime        boostback duration [s]
%     propBoostback   boostback propellant [kg] (planned)
%     aimBias         targeting bias for drag + entry burn [m]
%     entrySized      entry burn was sized and flown
%     propEntry       entry-burn propellant [kg]
%     qPeak           peak dynamic pressure over the descent [Pa]
%     entryLimitMet   qPeak <= p.entry.qMax in the plant (the sizing uses the
%                     guidance model, which under-predicts by a few %)
%     heatPeak        peak heating index sqrt(rho)*V^3 (relative units)
%     apogee          max altitude after boostback cutoff [m]
%     ignitionMissX   downrange miss at landing-burn ignition altitude [m]
%     ignitionSpeed   speed there [m/s]
%     propAtIgnition  propellant left above dry mass at ignition [kg]
%     landingMargin   propAtIgnition minus the ideal (impulsive) landing-burn
%                     propellant for ignitionSpeed [kg]
%     simTime         wall-clock time of the Simulink run [s]

m = struct('feasible', false, 'burnTime', NaN, 'propBoostback', NaN, ...
    'aimBias', NaN, 'entrySized', false, 'propEntry', NaN, 'qPeak', NaN, ...
    'entryLimitMet', false, 'heatPeak', NaN, 'apogee', NaN, ...
    'ignitionMissX', NaN, 'ignitionSpeed', NaN, 'propAtIgnition', NaN, ...
    'landingMargin', NaN, 'simTime', NaN);

try
    flight = simulate_mission(p);
catch err
    if strcmp(err.identifier, 'boostback_guidance:infeasible')
        return
    end
    rethrow(err);
end

t = flight.t; X = flight.X; ev = flight.ev;
after = t >= ev.tCutoff;

q = zeros(size(t)); heat = q;
for i = find(after)'
    [~, in] = dragAcceleration(X(i,:)', p);
    q(i) = in.q; heat(i) = in.heat;
end
upToIgnition = after & t <= ev.tIgnite;

m.feasible      = true;
m.burnTime      = flight.bb.duration;
m.propBoostback = flight.bb.propUsed;
m.aimBias       = flight.bb.aimBias;
m.entrySized    = flight.entrySized && ~isempty(ev.tEntry);
m.propEntry     = 0;
if m.entrySized
    mBefore = interp1(t, X(:,5), ev.tEntry(1) - 1e-6);
    m.propEntry = mBefore - flight.mCutoff;
end
m.qPeak         = max(q(upToIgnition));
m.entryLimitMet = m.qPeak <= p.entry.qMax;
m.heatPeak      = max(heat(upToIgnition));
m.apogee        = max(X(after, 2));
ign = flight.ignition;
m.ignitionMissX = ign(1) - p.target.x;
m.ignitionSpeed = norm(ign(3:4));
m.propAtIgnition = ign(5) - p.dryMass;
m.landingMargin = m.propAtIgnition - ign(5) * (1 - exp(-m.ignitionSpeed / (p.Isp * p.g0)));
m.simTime       = flight.simTime;
end

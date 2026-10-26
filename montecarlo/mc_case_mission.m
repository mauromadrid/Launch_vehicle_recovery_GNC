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
%   Flight loads (flight_loads: separation to landing-burn ignition):
%     qPeak           max dynamic pressure, max-Q [Pa]
%     maxQAltitude    altitude of max-Q [m];  maxQMach  Mach at max-Q
%     entryLimitMet   qPeak <= p.entry.qMax in the plant (the sizing uses the
%                     guidance model, which under-predicts by a few %)
%     nAxialMax       peak sensed/axial acceleration [g] (overall), and
%     nAxialBoostback, nAxialDescent  its peak per phase [g]
%     heatPeak        peak heating index sqrt(rho)*V^3 (relative units)
%     heatFluxMax     peak Sutton-Graves stagnation heat flux [W/m^2]
%                     (assumed nose radius: order of magnitude only)
%     heatLoad        integrated heat load [J/m^2]
%     apogee          max altitude after boostback cutoff [m]
%     ignitionMissX   downrange miss at landing-burn ignition altitude [m]
%     ignitionSpeed   speed there [m/s]
%     propAtIgnition  propellant left above dry mass at ignition [kg]
%     landingMargin   propAtIgnition minus the ideal (impulsive) landing-burn
%                     propellant for ignitionSpeed [kg]
%   Landing (in-flight SCvx guidance, re-planned):
%     touchdownSpeed  speed at ground contact [m/s]; touchdownVx, touchdownVy
%     touchdownMissX  downrange miss at ground contact [m]
%     propAtTouchdown propellant left above dry mass at touchdown [kg]
%     nLandingPlans   plans made in flight (1 at ignition + re-plans)
%     simTime         wall-clock time of the Simulink run [s]

m = struct('feasible', false, 'burnTime', NaN, 'propBoostback', NaN, ...
    'aimBias', NaN, 'entrySized', false, 'propEntry', NaN, 'qPeak', NaN, ...
    'maxQAltitude', NaN, 'maxQMach', NaN, 'entryLimitMet', false, ...
    'nAxialMax', NaN, 'nAxialBoostback', NaN, 'nAxialDescent', NaN, ...
    'heatPeak', NaN, 'heatFluxMax', NaN, 'heatLoad', NaN, 'apogee', NaN, ...
    'ignitionMissX', NaN, 'ignitionSpeed', NaN, 'propAtIgnition', NaN, ...
    'landingMargin', NaN, 'touchdownSpeed', NaN, 'touchdownVx', NaN, ...
    'touchdownVy', NaN, 'touchdownMissX', NaN, 'propAtTouchdown', NaN, ...
    'nLandingPlans', NaN, 'simTime', NaN);

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
loads = flight_loads(flight, p);

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
m.qPeak         = loads.maxQ;
m.maxQAltitude  = loads.maxQAltitude;
m.maxQMach      = loads.maxQMach;
m.entryLimitMet = m.qPeak <= p.entry.qMax;
m.nAxialMax     = loads.nAxialMax;
m.nAxialBoostback = loads.nAxialMaxBoostback;
m.nAxialDescent = loads.nAxialMaxDescent;
m.heatPeak      = loads.heatIdxMax;
m.heatFluxMax   = loads.heatFluxMax;
m.heatLoad      = loads.heatLoad;
m.apogee        = max(X(after, 2));
ign = flight.ignition;
m.ignitionMissX = ign(1) - p.target.x;
m.ignitionSpeed = norm(ign(3:4));
m.propAtIgnition = ign(5) - p.dryMass;
m.landingMargin = m.propAtIgnition - ign(5) * (1 - exp(-m.ignitionSpeed / (p.Isp * p.g0)));
td = flight.touchdown;
m.touchdownSpeed = norm(td(3:4));
m.touchdownVx   = td(3);
m.touchdownVy   = td(4);
m.touchdownMissX = td(1) - p.target.x;
m.propAtTouchdown = td(5) - p.dryMass;
m.nLandingPlans = numel(flight.landingPlanTimes);
m.simTime       = flight.simTime;
end

function m = mc_case_boostback(p)
%MC_CASE_BOOSTBACK Phase 2 Monte Carlo case: solve the boostback guidance
%   for parameter set p, fly it through the nonlinear EOM, coast to
%   landing-burn ignition altitude, and report scalar metrics.
%
%   An infeasible guidance problem is a legitimate *outcome* of a
%   dispersion (feasible = false), not an error; anything else that
%   throws is left to mc_run to record as an error.
%
%   Metrics:
%     feasible         guidance found a burn (logical)
%     propUsed         boostback propellant [kg]
%     propMargin       propellant left above the landing reserve [kg]
%     burnTime         boostback duration [s]
%     T_coast          coast duration used [s]
%     coastFallback    T_coast came from the fallback scan (logical)
%     relaxationGap    losslessness check (~0 expected)
%     solveTime        guidance wall-clock time [s]
%     apogee           max altitude after cutoff [m]
%     ignitionMissX    downrange miss at ignition altitude, replayed [m]
%     ignitionSpeed    speed at ignition altitude [m/s]

state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];

m = struct('feasible', false, 'propUsed', NaN, 'propMargin', NaN, ...
    'burnTime', NaN, 'T_coast', NaN, 'coastFallback', false, ...
    'relaxationGap', NaN, 'solveTime', NaN, 'apogee', NaN, ...
    'ignitionMissX', NaN, 'ignitionSpeed', NaN);

tSolve = tic;
try
    bb = boostback_guidance(state0, p);
catch err
    if strcmp(err.identifier, 'boostback_guidance:infeasible')
        m.solveTime = toc(tSolve);
        return
    end
    rethrow(err);
end
m.solveTime = toc(tSolve);

traj = simulate_boostback_coast(state0, bb, p);

m.feasible      = true;
m.propUsed      = bb.propUsed;
m.propMargin    = bb.cutoffState(5) - p.dryMass - p.landingPropReserve;
m.burnTime      = bb.duration;
m.T_coast       = bb.T_coast;
m.coastFallback = bb.T_coastSource == "fallback scan";
m.relaxationGap = bb.relaxationGap;
m.apogee        = max(traj.XB(:,2));
m.ignitionMissX = traj.ignition(1) - bb.rTarget(1);
m.ignitionSpeed = norm(traj.ignition(3:4));
end

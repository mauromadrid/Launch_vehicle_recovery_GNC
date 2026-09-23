function tests = test_boostback_guidance
%TEST_BOOSTBACK_GUIDANCE Checks on the convex boostback guidance: the
%   coast-duration estimate, constraint satisfaction and losslessness of
%   the SOCP solution, and that flying the profile through the nonlinear
%   EOM actually arrives at the landing-burn ignition point.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
p = vehicleParams();
testCase.TestData.p = p;
testCase.TestData.state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];
testCase.TestData.bb = boostback_guidance(testCase.TestData.state0, p);
end

function testCoastEstimateIsMinimumDeltaV(testCase)
p = testCase.TestData.p;
r0 = [p.sep.x; p.sep.y];
v0 = [p.sep.vx; p.sep.vy];
rT = [p.target.x; p.landingIgniteAltitude];
[Tc, vNeeded] = estimate_coast_duration(r0, v0, rT, p);

% vNeeded must actually coast from r0 to rT in Tc seconds
rEnd = r0 + vNeeded * Tc + 0.5 * [0; -p.g] * Tc^2;
testCase.verifyEqual(rEnd, rT, 'AbsTol', 1e-6);

% ...and Tc must be a local minimizer of the required delta-v
dvAt = @(T) norm([(rT(1) - r0(1)) / T; (rT(2) - r0(2) + 0.5*p.g*T^2) / T] - v0);
testCase.verifyLessThan(norm(vNeeded - v0), dvAt(Tc - 1));
testCase.verifyLessThan(norm(vNeeded - v0), dvAt(Tc + 1));
end

function testConvexificationIsLossless(testCase)
% The slack sigma must equal ||u|| at the optimum, otherwise the
% relaxed problem's solution isn't physically flyable.
testCase.verifyLessThan(testCase.TestData.bb.relaxationGap, 1e-4);
end

function testThrustAndPointingConstraints(testCase)
p = testCase.TestData.p;
bb = testCase.TestData.bb;
tol = 1e-3;
testCase.verifyGreaterThanOrEqual(bb.T_nominal, p.T_min_boostback * (1 - tol));
testCase.verifyLessThanOrEqual(bb.T_nominal, p.T_max_boostback * (1 + tol));

pointingDev = acos([cos(bb.theta) sin(bb.theta)] * bb.nRef);
testCase.verifyLessThanOrEqual(pointingDev, p.boostbackMaxDeviation + 1e-4);
end

function testLandingReserveIsKept(testCase)
p = testCase.TestData.p;
bb = testCase.TestData.bb;
testCase.verifyGreaterThanOrEqual(bb.cutoffState(5), ...
    (p.dryMass + p.landingPropReserve) * (1 - 1e-6));
end

function testPlannedCoastHitsTarget(testCase)
p = testCase.TestData.p;
bb = testCase.TestData.bb;
c = bb.cutoffState;
rEnd = c(1:2) + c(3:4) * bb.T_coast + 0.5 * [0; -p.g] * bb.T_coast^2;
% ~0.2 m residual is coneprog's default constraint tolerance (1e-6 in the
% km-scaled units, amplified by T_coast); tightening it much further makes
% the interior-point solver stall instead of converging.
testCase.verifyEqual(rEnd, bb.rTarget, 'AbsTol', 1);
end

function testNonlinearReplayReachesIgnitionPoint(testCase)
% Fly the ZOH profile as T = aCmd * m through threeDOF_EOM, then coast
% to ignition altitude: must arrive at the target within a few metres.
p = testCase.TestData.p;
bb = testCase.TestData.bb;
N = numel(bb.t);
dtNode = bb.duration / N;
opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-8);

s = testCase.TestData.state0;
for k = 1:N
    thrustFcn = @(t, x) deal(bb.aCmd(k) * x(5), bb.theta(k));
    [~, xk] = ode45(@(t, x) threeDOF_EOM(t, x, thrustFcn, p), [0 dtNode], s, opts);
    s = xk(end, :)';
end
% Absolute tolerances: coneprog satisfies the 40 dynamics equalities only
% to its tolerance (~0.03 m/s at cutoff worst case), and relative
% tolerances would fail spuriously on near-zero components (e.g. vx ~ 0
% when the boostback kills almost all horizontal speed).
testCase.verifyEqual(s(1:2), bb.cutoffState(1:2), 'AbsTol', 1);
testCase.verifyEqual(s(3:4), bb.cutoffState(3:4), 'AbsTol', 0.05);
testCase.verifyEqual(s(5), bb.cutoffState(5), 'RelTol', 1e-6);

coastOpts = odeset(opts, 'Events', @(t, x) altitudeEvent(t, x, p.landingIgniteAltitude));
[tB, xB] = ode45(@(t, x) threeDOF_EOM(t, x, @(~, ~) deal(0, 0), p), ...
                 [0 2 * bb.T_coast], s, coastOpts);
testCase.verifyEqual(xB(end, 1), bb.rTarget(1), 'AbsTol', 5);
testCase.verifyEqual(tB(end), bb.T_coast, 'AbsTol', 0.05);
end

function testDownrangeLandingSites(testCase)
% The landing site is a free parameter. Sites closer to the natural
% ballistic impact point need less reversal, so the burn must stay
% feasible and get cheaper than a return to the launch site. (Regression:
% a tight pointing cone about the estimated axis once made these
% infeasible.) The RTLS baseline is computed here, so the test doesn't
% depend on which landing site vehicleParams currently uses.
p = testCase.TestData.p;
p.target.x = 0;
propRTLS = boostback_guidance([], p).propUsed;
for xt = [100e3 200e3]
    p.target.x = xt;
    bb = boostback_guidance([], p);
    c = bb.cutoffState;
    xEnd = c(1) + c(3) * bb.T_coast;
    % coneprog's constraint tolerance is relative, so the residual grows
    % with target distance (~2 m at 200 km) -- still far inside what the
    % landing burn absorbs.
    testCase.verifyEqual(xEnd, xt, 'AbsTol', 5);
    testCase.verifyLessThan(bb.propUsed, propRTLS);
    testCase.verifyLessThan(bb.relaxationGap, 1e-4);
end
end

function testInfeasibleBudgetErrors(testCase)
% The original Phase 1 propellant load cannot reverse this separation
% state; the guidance must say so rather than return garbage.
p = testCase.TestData.p;
p.propMassAtSeparation = 3000;
p.m0 = p.dryMass + p.propMassAtSeparation;
testCase.verifyError(@() boostback_guidance([], p), 'boostback_guidance:infeasible');
end

function tests = test_entry_burn
%TEST_ENTRY_BURN Checks on the entry-burn sizing (guidance predictor +
%   root find) and on flying it in the plant.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
warning('off', 'aeroblks:aeroblkcoesa:AltMsg');
p = vehicleParams();
p.target.x = 0;
% A representative state at the burn start altitude: 60 km, 1450 m/s,
% 75 deg below horizontal, falling back toward the launch site.
V = 1450; gam = deg2rad(75);
testCase.TestData.state = [10e3; 60e3; -V*cos(gam); -V*sin(gam); 5900];
testCase.TestData.p = p;
end

function testInactiveLimitsNeedNoBurn(testCase)
p = testCase.TestData.p;
p.entry.qMax = Inf; p.entry.heatMaxRel = Inf; p.entry.vIgnitionMax = Inf;
eb = entry_burn_sizing(testCase.TestData.state, p);
testCase.verifyEqual(eb.dv, 0);
testCase.verifyTrue(eb.feasible);
testCase.verifyEqual(eb.binding, "none");
end

function testSizedBurnMeetsLimitInGuidanceModel(testCase)
% A reachable limit: pick one between the no-burn and the max-burn peaks,
% and check the root find lands on it (within the fzero tolerance).
p = testCase.TestData.p;
p.entry.heatMaxRel = Inf; p.entry.vIgnitionMax = Inf;
p.entry.qMax = Inf;
noBurn = entry_burn_sizing(testCase.TestData.state, p).noBurn;
p.entry.qMax = 0.7 * noBurn.qPeak;
eb = entry_burn_sizing(testCase.TestData.state, p);
testCase.verifyTrue(eb.feasible);
testCase.verifyEqual(eb.binding, "qMax");
testCase.verifyGreaterThan(eb.dv, 0);
testCase.verifyEqual(eb.predicted.qPeak, p.entry.qMax, 'RelTol', 0.01);
end

function testMostRestrictiveLimitWins(testCase)
p = testCase.TestData.p;
p.entry.qMax = Inf; p.entry.heatMaxRel = Inf; p.entry.vIgnitionMax = Inf;
noBurn = entry_burn_sizing(testCase.TestData.state, p).noBurn;
p.entry.qMax = 0.9 * noBurn.qPeak;          % mild
p.entry.heatMaxRel = 0.6;                   % strict
eb = entry_burn_sizing(testCase.TestData.state, p);
testCase.verifyEqual(eb.dv, max(eb.dvByLimit.qMax, eb.dvByLimit.heatMaxRel));
testCase.verifyLessThanOrEqual(eb.predicted.qPeak, p.entry.qMax * 1.01);
testCase.verifyLessThanOrEqual(eb.predicted.heatPeak, 0.6 * noBurn.heatPeak * 1.01);
end

function testUnreachableLimitIsInfeasibleAndKeepsReserve(testCase)
p = testCase.TestData.p;
p.entry.qMax = 1e3;                         % 1 kPa: impossible
eb = entry_burn_sizing(testCase.TestData.state, p);
testCase.verifyFalse(eb.feasible);
testCase.verifyTrue(isnan(eb.dvByLimit.qMax));
% best effort, but never below the landing reserve
testCase.verifyEqual(eb.mCutoff, p.dryMass + p.landingPropReserve, 'RelTol', 1e-9);
end

function testPlantBurnsPlannedPropellantRetrograde(testCase)
% Full flight with the entry burn enabled: the plant must cut off at the
% planned mass (rocket equation) and thrust opposite to the velocity.
p = testCase.TestData.p;
p.entry.enabled = true; p.entry.qMax = 150e3;
state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];
bb = boostback_guidance(state0, p);
tr = simulate_boostback_coast(state0, bb, p);
testCase.assertNotEmpty(tr.tEntry);
iEnd = find(tr.tB >= tr.tEntry(2), 1);
testCase.verifyEqual(tr.XB(iEnd, 5), tr.entry.mCutoff, 'RelTol', 1e-6);
burning = tr.TB > 0;
v = tr.XB(burning, 3:4);
thrustDir = [cos(tr.thetaB(burning)), sin(tr.thetaB(burning))];
testCase.verifyEqual(sum(thrustDir .* v, 2) ./ vecnorm(v, 2, 2), -ones(nnz(burning), 1), 'AbsTol', 1e-9);
end

function tests = test_simulate_mission
%TEST_SIMULATE_MISSION Checks on the main-simulation entry point (the
%   Simulink model run via simulate_mission) and the Monte Carlo case that
%   uses it. Each flight is a Simulink run (~10-15 s).
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
warning('off', 'pdg_landing_guidance:placeholder');
p = vehicleParams();
p.target.x = 0;                 % RTLS, whatever vehicleParams says locally
testCase.TestData.p = p;
testCase.TestData.flight = simulate_mission(p);
end

function testFliesTheParametersPassedIn(testCase)
% The model's InitFcn would rebuild p from vehicleParams(); the run must
% use the p passed in instead (this is what makes dispersions real).
p = testCase.TestData.p;
fl = testCase.TestData.flight;
testCase.verifyEqual(fl.p, p);
testCase.verifyEqual(fl.X(1,:)', [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0], 'RelTol', 1e-9);
testCase.verifyLessThan(abs(fl.ignition(1) - p.target.x), 150);
end

function testEntryBurnSizedInFlight(testCase)
fl = testCase.TestData.flight;
testCase.verifyTrue(fl.entrySized);
testCase.verifyNotEmpty(fl.ev.tEntry);
iAfter = find(fl.t > fl.ev.tEntry(2), 1);
testCase.verifyEqual(fl.X(iAfter, 5), fl.mCutoff, 'RelTol', 1e-6);
end

function testMonteCarloCaseMetrics(testCase)
m = mc_case_mission(testCase.TestData.p);
testCase.verifyTrue(m.feasible);
testCase.verifyTrue(m.entrySized);
testCase.verifyGreaterThan(m.propEntry, 0);
testCase.verifyLessThan(abs(m.ignitionMissX), 150);
testCase.verifyGreaterThan(m.landingMargin, 0);
end

function testInfeasibleBoostbackIsAnOutcome(testCase)
p = testCase.TestData.p;
p.propMassAtSeparation = 3000;
p = deriveParams(p);
m = mc_case_mission(p);
testCase.verifyFalse(m.feasible);
testCase.verifyTrue(isnan(m.ignitionMissX));
end

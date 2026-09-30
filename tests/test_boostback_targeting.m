function tests = test_boostback_targeting
%TEST_BOOSTBACK_TARGETING Checks on the aim-point bias loop that makes the
%   (vacuum) convex boostback account for drag and the entry burn.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
warning('off', 'aeroblks:aeroblkcoesa:AltMsg');
p = vehicleParams();
p.target.x = 0;
p.aero.dragEnabled = true;
p.entry.enabled = true;
p.entry.qMax = 150e3;
testCase.TestData.p = p;
testCase.TestData.state0 = [p.sep.x; p.sep.y; p.sep.vx; p.sep.vy; p.m0];
testCase.TestData.bb = boostback_targeting(testCase.TestData.state0, p);
end

function testVacuumIsUnbiased(testCase)
% No drag and no entry burn: nothing to correct, plain boostback_guidance.
p = testCase.TestData.p;
p.aero.dragEnabled = false; p.entry.enabled = false;
bb = boostback_targeting([], p);
ref = boostback_guidance([], p);
testCase.verifyEqual(bb.aimBias, 0);
testCase.verifyEqual(bb.duration, ref.duration);
end

function testConvergesInGuidanceModel(testCase)
bb = testCase.TestData.bb;
testCase.verifyTrue(bb.converged);
testCase.verifyLessThan(abs(bb.predictedMiss), 10);
testCase.verifyLessThanOrEqual(bb.iterations, 4);
testCase.verifyEqual(bb.rTarget(1), testCase.TestData.p.target.x);  % reports the real target
end

function testPlantMissShrinks(testCase)
% Flown in the COESA plant: the bias must remove most of the miss the
% unbiased plan has. What's left is guidance-model error (exponential vs
% COESA atmosphere), measured at ~50-70 m when this was written.
p = testCase.TestData.p;
s0 = testCase.TestData.state0;
unbiased = simulate_boostback_coast(s0, boostback_guidance(s0, p), p);
biased   = simulate_boostback_coast(s0, testCase.TestData.bb, p);
missUnbiased = abs(unbiased.ignition(1) - p.target.x);
missBiased   = abs(biased.ignition(1) - p.target.x);
testCase.verifyLessThan(missBiased, 150);
testCase.verifyLessThan(missBiased, 0.25 * missUnbiased);
end

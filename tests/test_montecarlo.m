function tests = test_montecarlo
%TEST_MONTECARLO Checks on the phase-agnostic Monte Carlo framework:
%   sampling, applying samples to the parameter struct, and the runner's
%   error handling. Uses a trivial case function, so it's fast and
%   independent of the guidance.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
testCase.TestData.p = vehicleParams();
testCase.TestData.catalog = mc_dispersion_catalog(testCase.TestData.p);
end

function testUniformSamplesStayInBounds(testCase)
spec = mc_select(testCase.TestData.catalog, ["speed", "fpa", "sep.y"]);
S = mc_generate_samples(spec, 500);
for j = 1:height(spec)
    x = S.(spec.Name(j));
    testCase.verifyGreaterThanOrEqual(x, spec.A(j));
    testCase.verifyLessThanOrEqual(x, spec.B(j));
end
end

function testLatinHypercubeHitsEveryStratum(testCase)
% The defining LHS property: each of the N equal-width strata of every
% variable contains exactly one sample.
spec = mc_select(testCase.TestData.catalog, ["speed", "fpa"]);
N = 50;
S = mc_generate_samples(spec, N, Method="lhs");
for j = 1:height(spec)
    u = (S.(spec.Name(j)) - spec.A(j)) / (spec.B(j) - spec.A(j));
    testCase.verifyEqual(sort(ceil(u * N)), (1:N)');
end
end

function testSeedMakesRunsReproducible(testCase)
spec = mc_select(testCase.TestData.catalog, ["speed", "fpa"]);
S1 = mc_generate_samples(spec, 20, Seed=7);
S2 = mc_generate_samples(spec, 20, Seed=7);
S3 = mc_generate_samples(spec, 20, Seed=8);
testCase.verifyEqual(S1, S2);
testCase.verifyNotEqual(S1, S3);
end

function testNormalIsTruncatedAtThreeSigma(testCase)
spec = mc_select(testCase.TestData.catalog, "Isp");
spec.Distribution = "normal"; spec.A = 300; spec.B = 5;
S = mc_generate_samples(spec, 2000, Method="random");
testCase.verifyGreaterThanOrEqual(S.Isp, 285);
testCase.verifyLessThanOrEqual(S.Isp, 315);
testCase.verifyEqual(mean(S.Isp), 300, 'AbsTol', 0.5);
end

function testApplySpeedAndFlightPathAngle(testCase)
p = testCase.TestData.p;
sample = table(1500, 30, 'VariableNames', {'speed', 'fpa'});
q = mc_apply_sample(p, sample);
testCase.verifyEqual(hypot(q.sep.vx, q.sep.vy), 1500, 'RelTol', 1e-12);
testCase.verifyEqual(atan2d(q.sep.vy, q.sep.vx), 30, 'AbsTol', 1e-10);
end

function testApplyKeepsOtherPolarComponentNominal(testCase)
p = testCase.TestData.p;
q = mc_apply_sample(p, table(40, 'VariableNames', {'fpa'}));
testCase.verifyEqual(hypot(q.sep.vx, q.sep.vy), hypot(p.sep.vx, p.sep.vy), 'RelTol', 1e-12);
end

function testApplyRecomputesDerivedParams(testCase)
p = testCase.TestData.p;
sample = table(5000, 250e3, 'VariableNames', {'dryMass', 'T_engine_max'});
q = mc_apply_sample(p, sample);
testCase.verifyEqual(q.m0, 5000 + p.propMassAtSeparation);
testCase.verifyEqual(q.T_max_boostback, p.nEnginesBoostback * 250e3);
testCase.verifyEqual(q.T_min_boostback, p.throttle_min * p.nEnginesBoostback * 250e3);
end

function testApplyRejectsUnknownField(testCase)
p = testCase.TestData.p;
testCase.verifyError(@() mc_apply_sample(p, table(1, 'VariableNames', {'sep.zz'})), ...
    'mc_apply_sample:unknownField');
end

function testSelectRejectsUnknownName(testCase)
testCase.verifyError(@() mc_select(testCase.TestData.catalog, "windSpeed"), ...
    'mc_select:unknownVariable');
end

function testRunnerRecordsErrorsAndContinues(testCase)
p = testCase.TestData.p;
spec = mc_select(testCase.TestData.catalog, "sep.y");
S = mc_generate_samples(spec, 10);
% Case function that fails above 70 km, otherwise returns the altitude
caseFcn = @(q) failAbove(q, 70e3);
R = mc_run(p, S, caseFcn, Verbose=false);
bad = S.("sep.y") > 70e3;
testCase.verifyEqual(R.status == "error", bad);
testCase.verifyEqual(R.alt(~bad), S.("sep.y")(~bad));
testCase.verifyTrue(all(isnan(R.alt(bad))));
end

function m = failAbove(q, limit)
if q.sep.y > limit
    error('test:tooHigh', 'too high');
end
m = struct('alt', q.sep.y);
end

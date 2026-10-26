function tests = test_flight_record
%TEST_FLIGHT_RECORD Checks on the flight-record pipeline (build, save,
%   load, plot) using a small synthetic flight -- no simulation needed.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
warning('off', 'aeroblks:aeroblkcoesa:AltMsg');
p = vehicleParams();
t = (0:1:40)';
n = numel(t);
X = [linspace(45e3, 50e3, n)', linspace(60e3, 3e3, n)', ...
     -100 * ones(n,1), linspace(-500, -300, n)', linspace(7000, 6000, n)'];
T = zeros(n,1); T(t <= 10) = 1.5e5; T(t >= 35) = 1.0e5;           % boostback, landing
theta = pi * ones(n,1); theta(t >= 35) = pi/2;
ev = struct('tCutoff', 10, 'tEntry', [], 'tIgnite', 35);
raw = struct('t', t, 'X', X, 'T', T, 'theta', theta, 'ev', ev);
testCase.TestData.p = p;
testCase.TestData.raw = raw;
testCase.TestData.rec = make_flight_record(raw, p, Name="unittest", Source="synthetic");
end

function testEveryCatalogSignalIsStoredWithUnits(testCase)
rec = testCase.TestData.rec;
defs = flight_signal_defs();
cols = string(rec.signals.Properties.VariableNames);
for d = defs'
    testCase.verifyTrue(ismember(d.name, cols), "missing signal " + d.name);
    k = find(cols == d.name);
    testCase.verifyEqual(string(rec.signals.Properties.VariableUnits{k}), d.unit);
end
testCase.verifyEqual(height(rec.signals), numel(testCase.TestData.raw.t));
end

function testDerivedSignals(testCase)
rec = testCase.TestData.rec; raw = testCase.TestData.raw;
testCase.verifyEqual(rec.signals.speed, vecnorm(raw.X(:,3:4), 2, 2), 'RelTol', 1e-12);
% thrust angle is NaN while the engine is off
testCase.verifyTrue(all(isnan(rec.signals.thrustAngle(raw.T == 0))));
% dynamic pressure is exactly 0.5*rho*V^2 with COESA density
[~, ~, ~, rho] = atmoscoesa(raw.X(:,2), 'None');
testCase.verifyEqual(rec.signals.dynPressure, 0.5 * rho(:) .* rec.signals.speed.^2, 'RelTol', 1e-12);
end

function testThrustLimitsFollowThePhase(testCase)
rec = testCase.TestData.rec; p = testCase.TestData.p; t = rec.signals.t;
lim = rec.limits.thrust;
testCase.verifyEqual(lim.upper(t <= 10), repmat(p.T_max_boostback, nnz(t <= 10), 1));
testCase.verifyEqual(lim.lower(t >= 35), repmat(p.T_min, nnz(t >= 35), 1));
testCase.verifyTrue(all(isnan(lim.upper(t > 10 & t < 35))));       % coast: no limit
testCase.verifyEqual(string(rec.signals.phase(t == 20)), "coast");
end

function testHeatingDisplayLimit(testCase)
% One display limit, p.loads.heatFluxMax, on both heating panels: the
% heating-index line converted back with the Sutton-Graves constant must
% equal the heat-flux line.
rec = testCase.TestData.rec; p = testCase.TestData.p;
testCase.verifyEqual(rec.limits.heatFlux.upper, repmat(p.loads.heatFluxMax, height(rec.signals), 1));
backToFlux = rec.limits.heatIndex.upper * 1.7415e-4 / sqrt(p.aero.effectiveNoseRadius);
testCase.verifyEqual(backToFlux, rec.limits.heatFlux.upper, 'RelTol', 1e-12);
end

function testEvents(testCase)
ev = testCase.TestData.rec.events;
testCase.verifyEqual(ev.name(1:2), ["Separation"; "Boostback cutoff"]);
testCase.verifyTrue(ismember("Landing-burn ignition", ev.name));
end

function testSaveLoadRoundTrip(testCase)
folder = tempname; mkdir(folder);
cleanup = onCleanup(@() rmdir(folder, 's'));
file = save_flight_record(testCase.TestData.rec, folder);
S = load(file, 'rec');
testCase.verifyEqual(S.rec.signals, testCase.TestData.rec.signals);
testCase.verifyEqual(S.rec.meta.name, "unittest");
end

function testPlotFromRecordAndUnknownPanel(testCase)
fig = plot_flight_record(testCase.TestData.rec);
set(fig, 'Visible', 'off');
close(fig);
testCase.verifyError(@() plot_flight_record(testCase.TestData.rec, Panels="notASignal"), ...
    'plot_flight_record:unknownSignal');
end

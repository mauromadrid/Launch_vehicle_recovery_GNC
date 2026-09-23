function tests = test_drag
%TEST_DRAG Checks on the aerodynamic drag model (dragAcceleration.m) and
%   its use in the equations of motion.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
testCase.TestData.p = vehicleParams();
end

function testDragOpposesVelocity(testCase)
p = testCase.TestData.p;
state = [0; 20e3; 300; -400; 5000];
aDrag = dragAcceleration(state, p);
testCase.verifyLessThan(dot(aDrag, state(3:4)), 0);
% exactly antiparallel: cross product of drag and velocity is zero
testCase.verifyEqual(aDrag(1)*state(4) - aDrag(2)*state(3), 0, 'AbsTol', 1e-9);
end

function testMagnitudeMatchesFormula(testCase)
% Independent recomputation at a subsonic point, where CD is on a known
% table segment.
p = testCase.TestData.p;
h = 5000; V = 150; m = 5000;
[~, a, ~, rho] = atmoscoesa(h, 'None');
mach = V / a;
CD = interp1(p.aero.mach, p.aero.CD, mach);
expected = 0.5 * rho * V^2 * CD * pi/4 * p.aero.diameter^2 / m;
[aDrag, info] = dragAcceleration([0; h; 0; -V; m], p);
testCase.verifyEqual(norm(aDrag), expected, 'RelTol', 1e-12);
testCase.verifyEqual(info.mach, mach, 'RelTol', 1e-12);
testCase.verifyEqual(info.q, 0.5 * rho * V^2, 'RelTol', 1e-12);
end

function testZeroAtRestAndWhenDisabled(testCase)
p = testCase.TestData.p;
testCase.verifyEqual(dragAcceleration([0; 1000; 0; 0; 5000], p), [0; 0]);
p.aero.dragEnabled = false;
testCase.verifyEqual(dragAcceleration([0; 1000; 300; -300; 5000], p), [0; 0]);
end

function testNegligibleAboveAtmosphere(testCase)
% Above the clamp altitude: finite (no NaN from COESA's extrapolation)
% and far below anything that matters.
p = testCase.TestData.p;
for h = [120e3 150e3 220e3]
    aDrag = dragAcceleration([0; h; 1500; 500; 5000], p);
    testCase.verifyTrue(all(isfinite(aDrag)));
    testCase.verifyLessThan(norm(aDrag), 1e-4);
end
end

function testEOMIncludesDrag(testCase)
% Unpowered: EOM acceleration = gravity + drag
p = testCase.TestData.p;
state = [0; 10e3; 200; -300; 5000];
dstate = threeDOF_EOM(0, state, @(~, ~) deal(0, 0), p);
aDrag = dragAcceleration(state, p);
testCase.verifyEqual(dstate(3:4), [0; -p.g] + aDrag, 'AbsTol', 1e-12);
end

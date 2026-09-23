function tests = test_threeDOF_EOM
%TEST_THREEDOF_EOM Basic sanity checks for the point-mass EOM.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
testCase.TestData.p = vehicleParams();
end

function testZeroThrustIsFreeFall(testCase)
% With zero thrust, horizontal velocity and mass must be conserved,
% and vertical acceleration must equal -g.
p = testCase.TestData.p;
state = [0; 10000; 200; 50; p.m0];
thrustFcn = @(t, s) deal(0, 0);

dstate = threeDOF_EOM(0, state, thrustFcn, p);

testCase.verifyEqual(dstate(1), state(3), 'AbsTol', 1e-9); % xdot = vx
testCase.verifyEqual(dstate(2), state(4), 'AbsTol', 1e-9); % ydot = vy
testCase.verifyEqual(dstate(3), 0, 'AbsTol', 1e-9);         % ax = 0
testCase.verifyEqual(dstate(4), -p.g, 'AbsTol', 1e-9);      % ay = -g
testCase.verifyEqual(dstate(5), 0, 'AbsTol', 1e-9);         % mdot = 0
end

function testThrustDepletesMass(testCase)
p = testCase.TestData.p;
state = [0; 5000; 100; 0; p.m0];
T = 0.5 * p.T_max;
thrustFcn = @(t, s) deal(T, pi/2);

dstate = threeDOF_EOM(0, state, thrustFcn, p);
expectedMdot = -T / (p.Isp * p.g0);

testCase.verifyEqual(dstate(5), expectedMdot, 'RelTol', 1e-9);
testCase.verifyLessThan(dstate(5), 0); % mass must decrease under thrust
end

function testThrustCutsOffAtDryMass(testCase)
p = testCase.TestData.p;
state = [0; 100; 0; -5; p.dryMass]; % already at dry mass
thrustFcn = @(t, s) deal(p.T_max, pi/2);

dstate = threeDOF_EOM(0, state, thrustFcn, p);

% Thrust should be zeroed out once mass reaches dry mass, so
% vertical acceleration should be exactly -g and mdot = 0.
testCase.verifyEqual(dstate(4), -p.g, 'AbsTol', 1e-9);
testCase.verifyEqual(dstate(5), 0, 'AbsTol', 1e-9);
end

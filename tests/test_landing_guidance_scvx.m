function tests = test_landing_guidance_scvx
%TEST_LANDING_GUIDANCE_SCVX Checks on the SCvx landing-burn guidance:
%   convergence, constraint satisfaction, losslessness, and that the plan
%   lands when flown through the guidance's own (drag) model.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
p = vehicleParams();
p.target.x = 0;
% Representative ignition state (nominal RTLS mission at 3 km)
testCase.TestData.state = [-40.2; 3000; -53.9; -350.3; 5225];
testCase.TestData.p = p;
testCase.TestData.ld = landing_guidance_scvx(testCase.TestData.state, p);
end

function testConverges(testCase)
ld = testCase.TestData.ld;
testCase.verifyTrue(ld.converged);
testCase.verifyLessThan(ld.iterations, 15);
testCase.verifyLessThan(ld.maxDefect, 1e-6);       % real (nonlinear) dynamics satisfied
end

function testLandsOnPadAtRest(testCase)
ld = testCase.TestData.ld; p = testCase.TestData.p;
testCase.verifyEqual(ld.r(:,end), [p.target.x; 0], 'AbsTol', 1e-2);
testCase.verifyEqual(ld.v(:,end), [0; 0], 'AbsTol', 1e-2);
testCase.verifyGreaterThanOrEqual(ld.m(end), p.dryMass);
end

function testConstraints(testCase)
ld = testCase.TestData.ld; p = testCase.TestData.p;
tilt = abs(ld.theta - pi/2);
testCase.verifyLessThanOrEqual(tilt, p.maxTiltAngle + 1e-4);
T = ld.aCmd .* ld.m(1:end-1)';
testCase.verifyGreaterThanOrEqual(T, p.T_min * (1 - 1e-3));
testCase.verifyLessThanOrEqual(T, p.T_max * (1 + 1e-3));
gs = abs(ld.r(1,:) - p.target.x) - ld.r(2,:) * cot(p.glideSlopeAngle);
testCase.verifyLessThanOrEqual(gs, 1e-3);
testCase.verifyLessThan(ld.relaxationGap, 1e-3);
end

function testPlanLandsInGuidanceModel(testCase)
% Fly the ZOH thrust-acceleration profile through guidance_model_eom
% (same drag model the plan used): must touch down slowly on the pad.
ld = testCase.TestData.ld; p = testCase.TestData.p;
c = p.Isp * p.g0; dt = ld.duration / numel(ld.t);
eom = @(s, a, th) guidance_model_eom(s, 0, p) + [0; 0; a*[cos(th); sin(th)]; -a*s(5)/c];
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @touchdownEvent);
s = testCase.TestData.state;
for k = 1:numel(ld.t)
    [~, X, te] = ode45(@(t, x) eom(x, ld.aCmd(k), ld.theta(k)), [0 dt], s, opts);
    s = X(end,:)';
    if ~isempty(te), break, end
end
testCase.verifyLessThan(abs(s(2)), 1);              % at the ground
testCase.verifyLessThan(abs(s(1) - p.target.x), 1);
testCase.verifyLessThan(norm(s(3:4)), 1);           % < 1 m/s
end

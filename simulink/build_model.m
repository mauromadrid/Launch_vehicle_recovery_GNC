%% BUILD_MODEL
% Programmatically constructs booster_return_3dof.slx from native
% Simulink blocks (no MATLAB Function block wrapping the dynamics --
% the equations of motion are visible as an actual block diagram).
%
% Re-running this script rebuilds the model from scratch, so it is the
% source of truth for the diagram, not the .slx file itself. Run this
% whenever the dynamics/guidance placeholders change structurally.
%
% Architecture:
%   VehicleDynamics subsystem   - planar 3DOF point-mass EOM (native
%                                  Integrator/Product/Trig/Sum blocks),
%                                  mirrors dynamics/threeDOF_EOM.m
%   GuidancePhaseLogic subsystem - boostback / coast / landing phase
%                                  sequencing via Switch blocks, driven
%                                  by elapsed time and altitude. The
%                                  boostback command is the convex-
%                                  guidance ZOH profile (1-D Lookup
%                                  Tables on time, flat interpolation),
%                                  flown as T = aCmd(t) * m
%   Top level                    - wiring, workspace logging (LogOut),
%                                  and a Stop-Simulation block that
%                                  halts the run at ground contact
%
% NOTE: the landing burn is still a fixed, non-optimized placeholder
% (guidance/pdg_landing_guidance.m, Phase 3) -- expect a hard "landing"
% until then. The boostback is the real convex solve (Phase 2).

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projRoot));
p  = vehicleParams();
bb = boostback_guidance([], p);
ld = pdg_landing_guidance([], p);

simDir = fullfile(projRoot, 'simulink');
if ~exist(simDir, 'dir'); mkdir(simDir); end

mdl = 'booster_return_3dof';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);
open_system(mdl);

%% ---- Vehicle Dynamics subsystem ----
dynPath = [mdl '/VehicleDynamics'];
add_block('simulink/Ports & Subsystems/Subsystem', dynPath);
try, delete_line(dynPath, 'In1/1', 'Out1/1'); catch, end
set_param([dynPath '/In1'], 'Name', 'T_in');
add_block('simulink/Sources/In1', [dynPath '/theta_in'], 'Port', '2');
set_param([dynPath '/Out1'], 'Name', 'x_out');
add_block('simulink/Sinks/Out1', [dynPath '/y_out'],  'Port', '2');
add_block('simulink/Sinks/Out1', [dynPath '/vx_out'], 'Port', '3');
add_block('simulink/Sinks/Out1', [dynPath '/vy_out'], 'Port', '4');
add_block('simulink/Sinks/Out1', [dynPath '/m_out'],  'Port', '5');

add_block('simulink/Continuous/Integrator', [dynPath '/Int_x'],  'InitialCondition', 'p.sep.x');
add_block('simulink/Continuous/Integrator', [dynPath '/Int_y'],  'InitialCondition', 'p.sep.y');
add_block('simulink/Continuous/Integrator', [dynPath '/Int_vx'], 'InitialCondition', 'p.sep.vx');
add_block('simulink/Continuous/Integrator', [dynPath '/Int_vy'], 'InitialCondition', 'p.sep.vy');
add_block('simulink/Continuous/Integrator', [dynPath '/Int_m'],  'InitialCondition', 'p.m0');

% Mass-depletion safeguard: gate thrust (and mass flow) to zero once
% the stage reaches dry mass -- mirrors the cutoff in threeDOF_EOM.m
add_block('simulink/Logic and Bit Operations/Relational Operator', [dynPath '/MassCutoff'], 'Operator', '>');
add_block('simulink/Sources/Constant', [dynPath '/DryMassConst'], 'Value', 'p.dryMass');
add_block('simulink/Signal Routing/Switch', [dynPath '/ThrustGate'], 'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
add_block('simulink/Sources/Constant', [dynPath '/ZeroConst'], 'Value', '0');

add_block('simulink/Math Operations/Divide', [dynPath '/T_over_m']);
add_block('simulink/Math Operations/Trigonometric Function', [dynPath '/CosTheta'], 'Operator', 'cos');
add_block('simulink/Math Operations/Trigonometric Function', [dynPath '/SinTheta'], 'Operator', 'sin');

add_block('simulink/Math Operations/Product', [dynPath '/Ax_Prod']);
add_block('simulink/Math Operations/Product', [dynPath '/Ay_Prod']);
add_block('simulink/Sources/Constant', [dynPath '/g_const'], 'Value', 'p.g');
add_block('simulink/Math Operations/Sum', [dynPath '/Ay_Sum'], 'Inputs', '+-');

add_block('simulink/Math Operations/Gain', [dynPath '/MdotGain'], 'Gain', '-1/(p.Isp*p.g0)');

add_line(dynPath, 'Int_m/1', 'MassCutoff/1');
add_line(dynPath, 'DryMassConst/1', 'MassCutoff/2');
add_line(dynPath, 'T_in/1', 'ThrustGate/1');
add_line(dynPath, 'MassCutoff/1', 'ThrustGate/2');
add_line(dynPath, 'ZeroConst/1', 'ThrustGate/3');

add_line(dynPath, 'ThrustGate/1', 'T_over_m/1');
add_line(dynPath, 'Int_m/1', 'T_over_m/2');

add_line(dynPath, 'theta_in/1', 'CosTheta/1');
add_line(dynPath, 'theta_in/1', 'SinTheta/1');

add_line(dynPath, 'T_over_m/1', 'Ax_Prod/1');
add_line(dynPath, 'CosTheta/1', 'Ax_Prod/2');

add_line(dynPath, 'T_over_m/1', 'Ay_Prod/1');
add_line(dynPath, 'SinTheta/1', 'Ay_Prod/2');

add_line(dynPath, 'Ay_Prod/1', 'Ay_Sum/1');
add_line(dynPath, 'g_const/1', 'Ay_Sum/2');

add_line(dynPath, 'ThrustGate/1', 'MdotGain/1');

add_line(dynPath, 'Ax_Prod/1', 'Int_vx/1');
add_line(dynPath, 'Ay_Sum/1', 'Int_vy/1');
add_line(dynPath, 'Int_vx/1', 'Int_x/1');
add_line(dynPath, 'Int_vy/1', 'Int_y/1');
add_line(dynPath, 'MdotGain/1', 'Int_m/1');

add_line(dynPath, 'Int_x/1',  'x_out/1');
add_line(dynPath, 'Int_y/1',  'y_out/1');
add_line(dynPath, 'Int_vx/1', 'vx_out/1');
add_line(dynPath, 'Int_vy/1', 'vy_out/1');
add_line(dynPath, 'Int_m/1',  'm_out/1');

Simulink.BlockDiagram.arrangeSystem(dynPath);

%% ---- Guidance / phase-sequencing subsystem ----
guidPath = [mdl '/GuidancePhaseLogic'];
add_block('simulink/Ports & Subsystems/Subsystem', guidPath);
try, delete_line(guidPath, 'In1/1', 'Out1/1'); catch, end
set_param([guidPath '/In1'], 'Name', 'y_in');
add_block('simulink/Sources/In1', [guidPath '/m_in'], 'Port', '2');
set_param([guidPath '/Out1'], 'Name', 'T_cmd');
add_block('simulink/Sinks/Out1', [guidPath '/theta_cmd'], 'Port', '2');

add_block('simulink/Sources/Clock', [guidPath '/Clock']);
add_block('simulink/Logic and Bit Operations/Relational Operator', [guidPath '/IsBoostback'], 'Operator', '<');
add_block('simulink/Sources/Constant', [guidPath '/BoostbackDur'], 'Value', 'bb.duration');

add_block('simulink/Logic and Bit Operations/Relational Operator', [guidPath '/IsBelowIgnite'], 'Operator', '<=');
add_block('simulink/Sources/Constant', [guidPath '/IgniteAlt'], 'Value', 'p.landingIgniteAltitude');

% Boostback: piecewise-constant (ZOH) profile from the convex solve.
% 'Flat' interpolation holds each node's value until the next breakpoint.
add_block('simulink/Lookup Tables/1-D Lookup Table', [guidPath '/BB_aCmd'], ...
    'Table', 'bb.aCmd', 'BreakpointsForDimension1', 'bb.t', ...
    'InterpMethod', 'Flat', 'ExtrapMethod', 'Clip');
add_block('simulink/Lookup Tables/1-D Lookup Table', [guidPath '/BB_Theta'], ...
    'Table', 'bb.theta', 'BreakpointsForDimension1', 'bb.t', ...
    'InterpMethod', 'Flat', 'ExtrapMethod', 'Clip');
add_block('simulink/Math Operations/Product', [guidPath '/BB_T']); % T = aCmd * m
add_block('simulink/Sources/Constant', [guidPath '/LD_T'], 'Value', 'ld.T_const');
add_block('simulink/Sources/Constant', [guidPath '/LD_Theta'], 'Value', 'ld.theta_const');
add_block('simulink/Sources/Constant', [guidPath '/ZeroT'], 'Value', '0');
add_block('simulink/Sources/Constant', [guidPath '/ZeroTheta'], 'Value', '0');

add_block('simulink/Signal Routing/Switch', [guidPath '/InnerSwitchT'],     'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
add_block('simulink/Signal Routing/Switch', [guidPath '/InnerSwitchTheta'], 'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
add_block('simulink/Signal Routing/Switch', [guidPath '/OuterSwitchT'],     'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
add_block('simulink/Signal Routing/Switch', [guidPath '/OuterSwitchTheta'], 'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');

add_line(guidPath, 'Clock/1', 'IsBoostback/1');
add_line(guidPath, 'Clock/1', 'BB_aCmd/1');
add_line(guidPath, 'Clock/1', 'BB_Theta/1');
add_line(guidPath, 'BB_aCmd/1', 'BB_T/1');
add_line(guidPath, 'm_in/1', 'BB_T/2');
add_line(guidPath, 'BoostbackDur/1', 'IsBoostback/2');

add_line(guidPath, 'y_in/1', 'IsBelowIgnite/1');
add_line(guidPath, 'IgniteAlt/1', 'IsBelowIgnite/2');

add_line(guidPath, 'LD_T/1', 'InnerSwitchT/1');
add_line(guidPath, 'IsBelowIgnite/1', 'InnerSwitchT/2');
add_line(guidPath, 'ZeroT/1', 'InnerSwitchT/3');

add_line(guidPath, 'LD_Theta/1', 'InnerSwitchTheta/1');
add_line(guidPath, 'IsBelowIgnite/1', 'InnerSwitchTheta/2');
add_line(guidPath, 'ZeroTheta/1', 'InnerSwitchTheta/3');

add_line(guidPath, 'BB_T/1', 'OuterSwitchT/1');
add_line(guidPath, 'IsBoostback/1', 'OuterSwitchT/2');
add_line(guidPath, 'InnerSwitchT/1', 'OuterSwitchT/3');

add_line(guidPath, 'BB_Theta/1', 'OuterSwitchTheta/1');
add_line(guidPath, 'IsBoostback/1', 'OuterSwitchTheta/2');
add_line(guidPath, 'InnerSwitchTheta/1', 'OuterSwitchTheta/3');

add_line(guidPath, 'OuterSwitchT/1', 'T_cmd/1');
add_line(guidPath, 'OuterSwitchTheta/1', 'theta_cmd/1');

Simulink.BlockDiagram.arrangeSystem(guidPath);

%% ---- Top-level wiring ----
add_line(mdl, 'GuidancePhaseLogic/1', 'VehicleDynamics/1'); % T_cmd    -> T_in
add_line(mdl, 'GuidancePhaseLogic/2', 'VehicleDynamics/2'); % theta_cmd-> theta_in
add_line(mdl, 'VehicleDynamics/2', 'GuidancePhaseLogic/1'); % y_out    -> y_in
add_line(mdl, 'VehicleDynamics/5', 'GuidancePhaseLogic/2'); % m_out    -> m_in

add_block('simulink/Signal Routing/Mux', [mdl '/LogMux'], 'Inputs', '7');
add_line(mdl, 'VehicleDynamics/1', 'LogMux/1');
add_line(mdl, 'VehicleDynamics/2', 'LogMux/2');
add_line(mdl, 'VehicleDynamics/3', 'LogMux/3');
add_line(mdl, 'VehicleDynamics/4', 'LogMux/4');
add_line(mdl, 'VehicleDynamics/5', 'LogMux/5');
add_line(mdl, 'GuidancePhaseLogic/1', 'LogMux/6');
add_line(mdl, 'GuidancePhaseLogic/2', 'LogMux/7');

add_block('simulink/Sinks/To Workspace', [mdl '/LogOut'], 'VariableName', 'simlog', 'SaveFormat', 'Structure With Time');
add_line(mdl, 'LogMux/1', 'LogOut/1');

%% ---- Stop-at-touchdown logic ----
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/TouchdownCheck'], 'Operator', '<=');
add_block('simulink/Sources/Constant', [mdl '/GroundConst'], 'Value', '0');
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/TimeGuard'], 'Operator', '>');
add_block('simulink/Sources/Clock', [mdl '/StopClock']);
add_block('simulink/Sources/Constant', [mdl '/EpsTime'], 'Value', '0.01');
add_block('simulink/Logic and Bit Operations/Logical Operator', [mdl '/StopAND'], 'Operator', 'AND');
add_block('simulink/Sinks/Stop Simulation', [mdl '/StopSim']);

add_line(mdl, 'VehicleDynamics/2', 'TouchdownCheck/1');
add_line(mdl, 'GroundConst/1', 'TouchdownCheck/2');
add_line(mdl, 'StopClock/1', 'TimeGuard/1');
add_line(mdl, 'EpsTime/1', 'TimeGuard/2');
add_line(mdl, 'TouchdownCheck/1', 'StopAND/1');
add_line(mdl, 'TimeGuard/1', 'StopAND/2');
add_line(mdl, 'StopAND/1', 'StopSim/1');

%% ---- Model configuration ----
% MaxStep: in the coast the motion is an exact parabola, so ode45's error
% control would otherwise take ~25 s steps -- accurate, but the logged
% signals become a few points joined by straight lines, and the solver
% steps far past events without zero-crossing detection (lookup-table
% node boundaries, the touchdown stop).
set_param(mdl, 'Solver', 'ode45', 'RelTol', '1e-9', 'AbsTol', '1e-9', ...
               'MaxStep', '0.5', 'StopTime', '600');

% Self-contained: (re)derive p, bb, ld whenever the model is simulated,
% regardless of what folder MATLAB's current working directory is.
initFcnStr = ['projRoot = fileparts(fileparts(get_param(bdroot,''FileName''))); ' ...
              'addpath(genpath(projRoot)); ' ...
              'p = vehicleParams(); ' ...
              'bb = boostback_guidance([], p); ' ...
              'ld = pdg_landing_guidance([], p);'];
set_param(mdl, 'InitFcn', initFcnStr);

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(simDir, [mdl '.slx']));
fprintf('Model built and saved to %s\n', fullfile(simDir, [mdl '.slx']));

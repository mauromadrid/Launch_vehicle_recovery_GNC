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
%                                  mirrors dynamics/threeDOF_EOM.m, with a
%                                  Drag subsystem (COESA atmosphere block,
%                                  CD(Mach) lookup) = dragAcceleration.m
%   GuidancePhaseLogic subsystem - boostback / coast / landing phase
%                                  sequencing via Switch blocks, driven
%                                  by elapsed time and altitude. The
%                                  boostback command is the convex-
%                                  guidance ZOH profile (1-D Lookup
%                                  Tables on time, flat interpolation),
%                                  flown as T = aCmd(t) * m, aimed by
%                                  boostback_targeting (drag + entry burn
%                                  bias). Entry burn: sized once at its
%                                  start altitude by a MATLAB Function
%                                  block in a Triggered Subsystem
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
bb = boostback_targeting([], p);
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
add_block('simulink/Math Operations/Sum', [dynPath '/Ax_Sum'], 'Inputs', '++');   % thrust + drag
add_block('simulink/Math Operations/Sum', [dynPath '/Ay_Sum'], 'Inputs', '+-+');  % thrust - g + drag

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

add_line(dynPath, 'Ax_Prod/1', 'Ax_Sum/1');
add_line(dynPath, 'Ax_Sum/1', 'Int_vx/1');
add_line(dynPath, 'Ay_Sum/1', 'Int_vy/1');
add_line(dynPath, 'Int_vx/1', 'Int_x/1');
add_line(dynPath, 'Int_vy/1', 'Int_y/1');
add_line(dynPath, 'MdotGain/1', 'Int_m/1');

add_line(dynPath, 'Int_x/1',  'x_out/1');
add_line(dynPath, 'Int_y/1',  'y_out/1');
add_line(dynPath, 'Int_vx/1', 'vx_out/1');
add_line(dynPath, 'Int_vy/1', 'vy_out/1');
add_line(dynPath, 'Int_m/1',  'm_out/1');

% ---- Drag subsystem (mirrors dynamics/dragAcceleration.m) ----
%   a_drag = -k * v,  k = 0.5 * rho * V * CD(Mach) * A_ref / m
% COESA 1976 atmosphere (Aerospace Blockset block), altitude clamped to
% [0, p.atmo.maxAltitude] because COESA's extrapolated temperature goes
% negative above ~175 km. The drag gain includes p.aero.dragEnabled so
% the same model runs as a vacuum plant.
dragPath = [dynPath '/Drag'];
add_block('simulink/Ports & Subsystems/Subsystem', dragPath);
try, delete_line(dragPath, 'In1/1', 'Out1/1'); catch, end
set_param([dragPath '/In1'], 'Name', 'vx');
add_block('simulink/Sources/In1', [dragPath '/vy'], 'Port', '2');
add_block('simulink/Sources/In1', [dragPath '/h'],  'Port', '3');
add_block('simulink/Sources/In1', [dragPath '/m'],  'Port', '4');
set_param([dragPath '/Out1'], 'Name', 'ax_drag');
add_block('simulink/Sinks/Out1', [dragPath '/ay_drag'], 'Port', '2');

add_block('simulink/Math Operations/Math Function', [dragPath '/Speed'], 'Operator', 'hypot');
add_block('simulink/Discontinuities/Saturation', [dragPath '/AltitudeClamp'], ...
    'LowerLimit', '0', 'UpperLimit', 'p.atmo.maxAltitude');
add_block('aerolibatmos/COESA Atmosphere Model', [dragPath '/COESA']);
add_block('simulink/Sinks/Terminator', [dragPath '/TermT']);
add_block('simulink/Sinks/Terminator', [dragPath '/TermP']);
add_block('simulink/Math Operations/Divide', [dragPath '/Mach']);
add_block('simulink/Lookup Tables/1-D Lookup Table', [dragPath '/CD_of_Mach'], ...
    'Table', 'p.aero.CD', 'BreakpointsForDimension1', 'p.aero.mach', ...
    'ExtrapMethod', 'Clip');
add_block('simulink/Math Operations/Product', [dragPath '/rhoVCD_over_m'], 'Inputs', '***/');
add_block('simulink/Math Operations/Gain', [dragPath '/NegHalfAref'], ...
    'Gain', '-0.5*p.aero.A_ref*p.aero.dragEnabled');
add_block('simulink/Math Operations/Product', [dragPath '/AxDrag']);
add_block('simulink/Math Operations/Product', [dragPath '/AyDrag']);

add_line(dragPath, 'vx/1', 'Speed/1');
add_line(dragPath, 'vy/1', 'Speed/2');
add_line(dragPath, 'h/1', 'AltitudeClamp/1');
add_line(dragPath, 'AltitudeClamp/1', 'COESA/1');
add_line(dragPath, 'COESA/1', 'TermT/1');
add_line(dragPath, 'COESA/3', 'TermP/1');
add_line(dragPath, 'Speed/1', 'Mach/1');
add_line(dragPath, 'COESA/2', 'Mach/2');          % speed of sound
add_line(dragPath, 'Mach/1', 'CD_of_Mach/1');
add_line(dragPath, 'COESA/4', 'rhoVCD_over_m/1'); % density
add_line(dragPath, 'Speed/1', 'rhoVCD_over_m/2');
add_line(dragPath, 'CD_of_Mach/1', 'rhoVCD_over_m/3');
add_line(dragPath, 'm/1', 'rhoVCD_over_m/4');
add_line(dragPath, 'rhoVCD_over_m/1', 'NegHalfAref/1');
add_line(dragPath, 'NegHalfAref/1', 'AxDrag/1');
add_line(dragPath, 'vx/1', 'AxDrag/2');
add_line(dragPath, 'NegHalfAref/1', 'AyDrag/1');
add_line(dragPath, 'vy/1', 'AyDrag/2');
add_line(dragPath, 'AxDrag/1', 'ax_drag/1');
add_line(dragPath, 'AyDrag/1', 'ay_drag/1');
Simulink.BlockDiagram.arrangeSystem(dragPath);

add_line(dynPath, 'Int_vx/1', 'Drag/1');
add_line(dynPath, 'Int_vy/1', 'Drag/2');
add_line(dynPath, 'Int_y/1',  'Drag/3');
add_line(dynPath, 'Int_m/1',  'Drag/4');
add_line(dynPath, 'Drag/1', 'Ax_Sum/2');
add_line(dynPath, 'Drag/2', 'Ay_Sum/3');

Simulink.BlockDiagram.arrangeSystem(dynPath);

%% ---- Guidance / phase-sequencing subsystem ----
guidPath = [mdl '/GuidancePhaseLogic'];
add_block('simulink/Ports & Subsystems/Subsystem', guidPath);
try, delete_line(guidPath, 'In1/1', 'Out1/1'); catch, end
set_param([guidPath '/In1'], 'Name', 'y_in');
add_block('simulink/Sources/In1', [guidPath '/m_in'], 'Port', '2');
add_block('simulink/Sources/In1', [guidPath '/x_in'],  'Port', '3');
add_block('simulink/Sources/In1', [guidPath '/vx_in'], 'Port', '4');
add_block('simulink/Sources/In1', [guidPath '/vy_in'], 'Port', '5');
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

% ---- Entry burn ----
% Sized once, when descending through p.entry.startAltitude: a Triggered
% Subsystem (falling edge of y - startAltitude, located by zero-crossing
% detection) runs a MATLAB Function block that calls the guidance's
% sizing (entry_burn_sizing, via entry_cutoff_mass) extrinsically on the
% measured state and holds the resulting cutoff mass. The burn is then
% native blocks: active while m > mCutoff, full thrust, retrograde.
add_block('simulink/Sources/Constant', [guidPath '/EntryStartAlt'], 'Value', 'p.entry.startAltitude');
add_block('simulink/Math Operations/Sum', [guidPath '/AboveEntryStart'], 'Inputs', '+-');
add_block('simulink/Signal Routing/Mux', [guidPath '/StateMux'], 'Inputs', '5');

trigPath = [guidPath '/EntryBurnSizing'];
add_block('simulink/Ports & Subsystems/Triggered Subsystem', trigPath);
try, delete_line(trigPath, 'In1/1', 'Out1/1'); catch, end
set_param([trigPath '/Trigger'], 'TriggerType', 'falling');
set_param([trigPath '/In1'], 'Name', 'state');
set_param([trigPath '/Out1'], 'Name', 'mCutoff');
add_block('simulink/Sinks/Out1', [trigPath '/sized'], 'Port', '2');
fcnPath = [trigPath '/size_entry_burn'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcnPath);
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', fcnPath);
chart.Script = strjoin({ ...
    'function [mCutoff, sized] = size_entry_burn(state, p)'
    '%SIZE_ENTRY_BURN Runs once, when the stage descends through the entry'
    '%   start altitude: sizes the entry burn from the measured state with'
    '%   the guidance''s own predictor (entry_burn_sizing, MATLAB code run'
    '%   extrinsically) and returns the engine-cutoff mass.'
    'coder.extrinsic(''entry_cutoff_mass'');'
    'mCutoff = 0;'
    'sized = 0;'
    'if p.entry.enabled'
    '    mCutoff = entry_cutoff_mass(state, p);'
    '    sized = 1;'
    'end'
    }, newline);
pData = chart.find('-isa', 'Stateflow.Data', 'Name', 'p');
pData.Scope = 'Parameter';         % the workspace struct p (set by InitFcn)
pData.Tunable = false;
add_line(trigPath, 'state/1', 'size_entry_burn/1');
add_line(trigPath, 'size_entry_burn/1', 'mCutoff/1');
add_line(trigPath, 'size_entry_burn/2', 'sized/1');
Simulink.BlockDiagram.arrangeSystem(trigPath);

add_block('simulink/Logic and Bit Operations/Relational Operator', [guidPath '/AboveCutoffMass'], 'Operator', '>');
add_block('simulink/Logic and Bit Operations/Logical Operator', [guidPath '/EntryActive'], 'Operator', 'AND');
add_block('simulink/Sources/Constant', [guidPath '/EntryT'], 'Value', 'p.T_max_entry');
add_block('simulink/Math Operations/Gain', [guidPath '/NegVx'], 'Gain', '-1');
add_block('simulink/Math Operations/Gain', [guidPath '/NegVy'], 'Gain', '-1');
add_block('simulink/Math Operations/Trigonometric Function', [guidPath '/RetroAngle'], 'Operator', 'atan2');
add_block('simulink/Signal Routing/Switch', [guidPath '/EntrySwitchT'],     'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
add_block('simulink/Signal Routing/Switch', [guidPath '/EntrySwitchTheta'], 'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');

add_line(guidPath, 'y_in/1', 'AboveEntryStart/1');
add_line(guidPath, 'EntryStartAlt/1', 'AboveEntryStart/2');
add_line(guidPath, 'AboveEntryStart/1', 'EntryBurnSizing/Trigger');
add_line(guidPath, 'x_in/1',  'StateMux/1');
add_line(guidPath, 'y_in/1',  'StateMux/2');
add_line(guidPath, 'vx_in/1', 'StateMux/3');
add_line(guidPath, 'vy_in/1', 'StateMux/4');
add_line(guidPath, 'm_in/1',  'StateMux/5');
add_line(guidPath, 'StateMux/1', 'EntryBurnSizing/1');
add_line(guidPath, 'm_in/1', 'AboveCutoffMass/1');
add_line(guidPath, 'EntryBurnSizing/1', 'AboveCutoffMass/2');
add_line(guidPath, 'EntryBurnSizing/2', 'EntryActive/1');
add_line(guidPath, 'AboveCutoffMass/1', 'EntryActive/2');
add_line(guidPath, 'vy_in/1', 'NegVy/1');
add_line(guidPath, 'vx_in/1', 'NegVx/1');
add_line(guidPath, 'NegVy/1', 'RetroAngle/1');   % atan2(-vy, -vx): opposite to velocity
add_line(guidPath, 'NegVx/1', 'RetroAngle/2');
add_line(guidPath, 'EntryT/1', 'EntrySwitchT/1');
add_line(guidPath, 'EntryActive/1', 'EntrySwitchT/2');
add_line(guidPath, 'ZeroT/1', 'EntrySwitchT/3');
add_line(guidPath, 'RetroAngle/1', 'EntrySwitchTheta/1');
add_line(guidPath, 'EntryActive/1', 'EntrySwitchTheta/2');
add_line(guidPath, 'ZeroTheta/1', 'EntrySwitchTheta/3');

% Landing burn below ignition altitude, else entry burn (if active) / coast
add_line(guidPath, 'LD_T/1', 'InnerSwitchT/1');
add_line(guidPath, 'IsBelowIgnite/1', 'InnerSwitchT/2');
add_line(guidPath, 'EntrySwitchT/1', 'InnerSwitchT/3');

add_line(guidPath, 'LD_Theta/1', 'InnerSwitchTheta/1');
add_line(guidPath, 'IsBelowIgnite/1', 'InnerSwitchTheta/2');
add_line(guidPath, 'EntrySwitchTheta/1', 'InnerSwitchTheta/3');

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
add_line(mdl, 'VehicleDynamics/1', 'GuidancePhaseLogic/3'); % x_out    -> x_in
add_line(mdl, 'VehicleDynamics/3', 'GuidancePhaseLogic/4'); % vx_out   -> vx_in
add_line(mdl, 'VehicleDynamics/4', 'GuidancePhaseLogic/5'); % vy_out   -> vy_in

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
              'bb = boostback_targeting([], p); ' ...
              'ld = pdg_landing_guidance([], p);'];
set_param(mdl, 'InitFcn', initFcnStr);

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(simDir, [mdl '.slx']));
fprintf('Model built and saved to %s\n', fullfile(simDir, [mdl '.slx']));

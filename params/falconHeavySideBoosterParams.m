function p = falconHeavySideBoosterParams(sep, propMassAtSeparation)
%FALCONHEAVYSIDEBOOSTERPARAMS Falcon Heavy side-booster (Falcon 9 first
%   stage) parameters for sanity-checking the boostback guidance at real
%   scale, on top of vehicleParams().
%
%   Sourced (public):
%     - Dry mass 22.2 t per core, Merlin 1D 981 kN / 311 s in vacuum,
%       sea-level throttle range 419-816 kN (~51% minimum):
%       https://en.wikipedia.org/wiki/Falcon_Heavy ,
%       https://en.wikipedia.org/wiki/SpaceX_Merlin
%     - Timeline: BECO ~T+2:24-2:29, boostback "just over one minute",
%       side-booster landing ~T+7:40-7:58 at LZ-1/LZ-2:
%       NASA Roman launch blog (2026-08-30); FH demo flight timeline.
%   NOT sourced (not published) -- passed in / swept by the caller:
%     - BECO state (position, velocity) and propellant remaining at BECO.
%   Assumed from general knowledge of Falcon operations:
%     - 3-engine boostback, vacuum Isp (burn happens at ~60-120 km).
%
%   sep  - struct with fields x, y, vx, vy at BECO [m, m/s]
%   propMassAtSeparation - propellant remaining at BECO [kg]
%
%   Planar model: LZ-1/2 are offset from the pads mostly cross-range,
%   which this 2D model can't represent; they're placed at the launch
%   site's downrange position (target.x = 0).

p = vehicleParams();

p.Isp               = 311;      % [s] Merlin 1D vacuum
p.T_engine_max      = 981e3;    % [N] Merlin 1D vacuum
p.throttle_min      = 419 / 816;
p.nEnginesBoostback = 3;
p.nEnginesLanding   = 1;

p.dryMass = 22.2e3;
p.propMassAtSeparation = propMassAtSeparation;
p.landingPropReserve = 0;      % caller decides; 0 = boostback need alone

p.sep = sep;
p.target.x = 0;

p = deriveParams(p);
end

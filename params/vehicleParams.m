function p = vehicleParams()
%VEHICLEPARAMS Representative parameters for the booster return study.
%   These are illustrative, order-of-magnitude values for a small
%   reusable first stage. They are NOT sourced from any specific
%   vehicle's real performance data and should be refined/justified as
%   the project matures.

% --- Environment ---
p.g0 = 9.80665;   % [m/s^2] standard gravity, used for Isp definition
p.g  = 9.80665;   % [m/s^2] local gravity (flat-Earth, constant-g
                   %         approximation for Phase 1; revisit if the
                   %         altitude range grows large enough to
                   %         matter, e.g. g = g0*(Re/(Re+y))^2)

% --- Propulsion ---
p.Isp             = 283;     % [s] representative sea-level Isp (LOX/RP-1-class)
p.nEnginesLanding   = 1;     % number of engines used for the landing burn
p.nEnginesBoostback = 1;     % number of engines used for the boostback burn
p.T_engine_max    = 200e3;   % [N] max thrust per engine
p.throttle_min    = 0.40;    % minimum throttle fraction (deep-throttling limit)
% T_min/T_max (landing) and T_min/T_max_boostback: see deriveParams()

% --- Mass properties (at stage separation) ---
p.dryMass              = 4000;  % [kg] structural/dry mass of the stage
p.propMassAtSeparation = 8000;  % [kg] propellant remaining for
                                  %      boostback + coast + landing.
                                  %      Sized for the boostback: from
                                  %      this separation state the burn
                                  %      needs ~1.6 km/s impulsive dv
                                  %      (more with gravity losses; ~6.1 t
                                  %      for RTLS), so the original
                                  %      3000 kg (1.55 km/s total) is
                                  %      infeasible.
% p.m0 (mass at separation): see deriveParams()

% --- Separation state (illustrative) ---
% Planar frame: x = downrange distance from the launch site, y = altitude.
% The stage separates while still moving away from the launch site.
p.sep.x  = 45e3;  % [m]   downrange position at separation
p.sep.y  = 65e3;  % [m]   altitude at separation
p.sep.vx = 1428;  % [m/s] downrange velocity at separation
p.sep.vy = 824;   % [m/s] vertical velocity at separation (still ascending)

% --- Landing target ---
% The landing site is independent of the launch site (x = 0): x = 0 is a
% return-to-launch-site (RTLS) landing zone, a large positive x is a
% downrange droneship. Everything downstream (T_coast estimate, boostback
% terminal constraint) reads p.target.x, so this is the only knob.
p.target.x  = 0;   % [m]   downrange position of the landing site
p.target.y  = 0;   % [m]   ground level
p.target.vx = 0;   % [m/s] touchdown velocity target
p.target.vy = -2;  % [m/s] small controlled sink rate at touchdown

% --- Landing burn ignition (Phase 1 open-loop / block-diagram trigger) ---
p.landingIgniteAltitude = 3000;  % [m] altitude at which the landing
                                    %     burn ignites in the (still
                                    %     placeholder) phase-sequencing
                                    %     logic

% --- Aerodynamics (Phase 4) ---
% Drag only (no lift, zero angle of attack): the stage descends engine-
% first, so the relevant drag is that of a blunt base leading.
%   D = 0.5 * rho * V^2 * CD(Mach) * A_ref, opposite to the velocity.
% The CD(Mach) curve is ILLUSTRATIVE: the generic blunt-body trend
% (subsonic ~0.8-0.9, transonic rise, supersonic plateau below the
% stagnation-pressure coefficient, 1.84 at the hypersonic limit for a
% flat face). Real tail-first booster data comes from wind tunnel + CFD
% databases, e.g. DLR's CALLISTO and the EU RETALT project.
p.aero.dragEnabled = true;       % false = vacuum plant (for comparisons/tests)
p.aero.diameter    = 1.5;        % [m] reference diameter (A_ref: deriveParams)
p.aero.mach = [0    0.5  0.8  1.0  1.2  1.5  2.0  3.0  5.0  10  ];
p.aero.CD   = [0.80 0.82 0.90 1.20 1.45 1.50 1.45 1.35 1.30 1.28];

% --- Atmosphere ---
% Plant ("truth"): COESA 1976 via atmoscoesa / the Aerospace Blockset
% COESA block. Its extrapolation above 86 km lets temperature fall
% linearly (negative at ~175 km), so the altitude fed to it is clamped at
% maxAltitude, where density (~3e-14 kg/m^3) already makes drag zero.
p.atmo.maxAltitude = 150e3;      % [m]
% Guidance model: exponential rho = rho0*exp(-h/H), least-squares fit to
% COESA over 0-80 km (density within +-23%). Deliberately simpler than the
% plant, as real onboard models are; used by the drag bias estimator.
p.atmo.rho0 = 1.4168;            % [kg/m^3]
p.atmo.H    = 7008;              % [m] scale height
p.atmo.soundSpeed = 300;         % [m/s] guidance model's constant speed of
                                 %       sound (Mach for CD); COESA ranges
                                 %       ~275-340 m/s below 86 km

% --- Entry burn (Phase 4) ---
% Retrograde burn (thrust opposite the velocity) at full thrust, started
% when descending through startAltitude. Its delta-v is sized online, at
% burn start, from the measured state: the minimum delta-v for which the
% guidance model predicts every active limit is met (entry_burn_sizing).
% Limits are ILLUSTRATIVE; set a limit to Inf to deactivate it.
p.entry.enabled       = true;
p.entry.startAltitude = 60e3;    % [m]
p.nEnginesEntry       = 1;       % (T_max_entry: deriveParams)
p.entry.qMax          = 150e3;   % [Pa]  peak dynamic pressure after the burn
                                 %       (reachable for RTLS; RTLS can't get
                                 %       below ~104 kPa with this vehicle)
p.entry.heatMaxRel    = Inf;     % [-]   peak heating index sqrt(rho)*V^3,
                                 %       relative to the same descent with
                                 %       no entry burn (e.g. 0.5 = halve it)
p.entry.vIgnitionMax  = Inf;     % [m/s] speed at landing-burn ignition
p.entry.limitMargin   = 0.10;    % [-]   size the burn to limit/(1+margin) on
                                 %       the absolute limits (q, ignition
                                 %       speed): the guidance atmosphere
                                 %       under-predicts peak q by ~3-8%

% --- Constraints (used by the guidance modules, Phases 2-3) ---
p.maxTiltAngle    = deg2rad(20);  % [rad] landing burn: max thrust angle
                                    %       from the vehicle's vertical axis
p.glideSlopeAngle = deg2rad(10);  % [rad] min angle of the glide-slope
                                    %       cone above the landing site

% --- Boostback guidance (Phase 2) ---
p.boostbackNodes     = 40;    % ZOH discretization intervals over the burn
p.boostbackMaxDeviation = deg2rad(90); % [rad] max thrust angle from the
                                %      boostback delta-v direction. 90 deg
                                %      = half-space (never thrust against
                                %      the manoeuvre). The vehicle flips
                                %      freely during boostback, so a tight
                                %      cone about this *estimated* axis is
                                %      not physical: at 20 deg it forced
                                %      ~70% extra fuel for downrange
                                %      landing sites and broke the T_coast
                                %      estimate there.
p.landingPropReserve = 800;   % [kg] propellant the boostback must leave
                                %      for the landing burn (floor on the
                                %      burn-cutoff mass above dryMass).
                                %      ~500 m/s of dv at dry mass: sized
                                %      for a landing from near-terminal
                                %      velocity, i.e. assumes Phase 4
                                %      drag. In vacuum the coast reaches
                                %      ignition altitude at ~1.8 km/s.

p = deriveParams(p);
end

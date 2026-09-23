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
                                  %      needs ~1.85 km/s impulsive dv
                                  %      (more with gravity losses), so
                                  %      the original 3000 kg (1.55 km/s
                                  %      total) made it infeasible.
% p.m0 (mass at separation): see deriveParams()

% --- Separation state (illustrative) ---
% Planar frame: x = downrange distance from the launch site, y = altitude.
% The stage separates while still moving away from the launch site.
p.sep.x  = 45e3;  % [m]   downrange position at separation
p.sep.y  = 65e3;  % [m]   altitude at separation
p.sep.vx = 1600;  % [m/s] downrange velocity at separation
p.sep.vy = 400;   % [m/s] vertical velocity at separation (still ascending)

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
                                %      ignition altitude at ~1.6 km/s.

p = deriveParams(p);
end

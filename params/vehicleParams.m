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
p.nEnginesLanding = 1;       % number of engines used for the landing burn
p.T_engine_max    = 200e3;   % [N] max thrust per engine
p.throttle_min    = 0.40;    % minimum throttle fraction (deep-throttling limit)
p.T_max = p.nEnginesLanding * p.T_engine_max;
p.T_min = p.throttle_min * p.T_max;

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
p.m0 = p.dryMass + p.propMassAtSeparation;

% --- Separation state (illustrative) ---
% Planar frame: x = downrange distance from the launch site, y = altitude.
% The stage separates while still moving away from the launch site.
p.sep.x  = 45e3;  % [m]   downrange position at separation
p.sep.y  = 65e3;  % [m]   altitude at separation
p.sep.vx = 1600;  % [m/s] downrange velocity at separation
p.sep.vy = 400;   % [m/s] vertical velocity at separation (still ascending)

% --- Landing target ---
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
p.maxTiltAngle    = deg2rad(20);  % [rad] max thrust pointing angle from
                                    %       the vehicle's vertical axis
p.glideSlopeAngle = deg2rad(10);  % [rad] min angle of the glide-slope
                                    %       cone above the landing site

% --- Boostback guidance (Phase 2) ---
p.boostbackNodes     = 40;    % ZOH discretization intervals over the burn
p.landingPropReserve = 800;   % [kg] propellant the boostback must leave
                                %      for the landing burn (floor on the
                                %      burn-cutoff mass above dryMass).
                                %      ~500 m/s of dv at dry mass: sized
                                %      for a landing from near-terminal
                                %      velocity, i.e. assumes Phase 4
                                %      drag. In vacuum the coast reaches
                                %      ignition altitude at ~1.3 km/s.
end

function thrustProfile = pdg_landing_guidance(state0, p) %#ok<INUSD>
%PDG_LANDING_GUIDANCE Placeholder for the convex powered-descent
%   guidance solve (Phase 3).
%
%   This will implement the lossless-convexification-based
%   minimum-fuel powered descent guidance problem (Açıkmeşe & Ploen,
%   and the "G-FOLD" family of algorithms), solved as a second-order
%   cone program via MATLAB's coneprog:
%
%       minimize    fuel consumption (~ integral of thrust magnitude)
%       subject to  translational dynamics (double integrator + mass)
%                   T_min <= ||T(t)|| <= T_max  (relaxed via slack Gamma)
%                   thrust pointing angle <= maxTiltAngle
%                   glide-slope constraint (stay above the landing cone)
%                   boundary conditions at t0 and tf (target state)
%
%   For now this returns a fixed placeholder burn so the full pipeline
%   can be run end-to-end before the optimizer is implemented.
%
%   TODO (Phase 3): implement the SOCP formulation and solve with
%   coneprog; see Açıkmeşe & Ploen (2007) and Blackmore et al. (2010).

warning('pdg_landing_guidance:placeholder', ...
    'Using placeholder fixed-burn landing profile -- convex PDG solve not yet implemented.');

thrustProfile.T_const     = p.T_min + 0.5 * (p.T_max - p.T_min);
thrustProfile.theta_const = pi/2; % straight up, opposing gravity
thrustProfile.duration    = 20; % [s]
end

function ds = guidance_model_eom(s, T, p)
%GUIDANCE_MODEL_EOM The guidance's own (onboard) descent model: point
%   mass, retrograde thrust T [N], gravity, and drag with the onboard
%   atmosphere table (guidance_density), a constant speed of sound
%   (p.atmo.soundSpeed) for Mach, and the CD(Mach) table.
%
%   Deliberately simpler than the plant (threeDOF_EOM + COESA): it's what
%   the flight computer predicts with, so plant-vs-model differences are
%   real guidance errors. Drag is off when p.aero.dragEnabled is false
%   (the guidance then believes it flies in vacuum, like the plant).
%
%   s = [x; y; vx; vy; m]

v = s(3:4); V = norm(v); m = s(5);
aDrag = [0; 0];
if p.aero.dragEnabled
    rho = guidance_density(s(2), p);
    CD = interp1(p.aero.mach, p.aero.CD, min(V / p.atmo.soundSpeed, p.aero.mach(end)));
    aDrag = -(0.5 * rho * V * CD * p.aero.A_ref / m) * v;
end
aThrust = [0; 0];
if T > 0 && V > 0
    aThrust = -(T / m) * v / V;
end
ds = [v; aThrust + [0; -p.g] + aDrag; -T / (p.Isp * p.g0)];
end

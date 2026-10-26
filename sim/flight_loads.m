function L = flight_loads(flight, p)
%FLIGHT_LOADS Flight-loads time histories and peaks along a simulated
%   mission (simulate_mission output), from separation to touchdown.
%
%   L = flight_loads(flight, p)
%
%   Window: the whole flight (boostback, coast, entry burn, drag-braked
%   descent, landing burn). "Descent" peaks include the landing burn.
%
%   Quantities (point-mass model, see notes):
%     q        dynamic pressure 0.5*rho*V^2 [Pa]
%     nAxial   sensed (non-gravitational) acceleration |T/m*e_T + a_drag|
%              in g -- what an accelerometer reads. Thrust and drag both act
%              along the vehicle axis in this model, so this is the axial
%              load factor.
%     heatIdx  relative heating index sqrt(rho)*V^3
%     heatFlux Sutton-Graves stagnation heat flux
%              qdot = 1.7415e-4 * sqrt(rho / Rn) * V^3   [W/m^2]
%              with an ASSUMED effective nose radius Rn =
%              p.aero.effectiveNoseRadius: order-of-magnitude only (the
%              stage leads with a blunt engine section, not a sphere)
%     heatLoad time integral of heatFlux [J/m^2]
%
%   L fields: t, q, nAxial, heatFlux (time histories) and the peaks:
%     maxQ [Pa], maxQAltitude [m], maxQMach, maxQTime [s]
%     nAxialMax [g] (overall), nAxialMaxBoostback, nAxialMaxDescent [g]
%     heatFluxMax [W/m^2], heatLoad [J/m^2], heatIdxMax

t = flight.t; X = flight.X; T = flight.T; th = flight.theta;

n = numel(t);
q = zeros(n,1); nAx = q; heatIdx = q; mach = q; rho = q;
for i = 1:n
    [aDrag, in] = dragAcceleration(X(i,:)', p);
    aThrust = T(i) / X(i,5) * [cos(th(i)); sin(th(i))];
    nAx(i) = norm(aThrust + aDrag) / p.g0;
    q(i) = in.q; heatIdx(i) = in.heat; mach(i) = in.mach; rho(i) = in.rho;
end
V = vecnorm(X(:,3:4), 2, 2);
heatFlux = 1.7415e-4 * sqrt(rho / p.aero.effectiveNoseRadius) .* V.^3;

[L.maxQ, iq] = max(q);
L.maxQAltitude = X(iq, 2);
L.maxQMach = mach(iq);
L.maxQTime = t(iq);
L.nAxialMax = max(nAx);
isBoostback = t <= flight.ev.tCutoff;
L.nAxialMaxBoostback = max([nAx(isBoostback); 0]);
L.nAxialMaxDescent = max([nAx(~isBoostback); 0]);
L.heatFluxMax = max(heatFlux);
L.heatLoad = trapz(t, heatFlux);
L.heatIdxMax = max(heatIdx);
L.t = t; L.q = q; L.nAxial = nAx; L.heatFlux = heatFlux; L.altitude = X(:,2);
end

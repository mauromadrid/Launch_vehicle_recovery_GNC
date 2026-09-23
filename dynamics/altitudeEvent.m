function [value, isterminal, direction] = altitudeEvent(~, state, yEvent)
%ALTITUDEEVENT ODE event function that stops integration when descending
%   through altitude yEvent (e.g. landing-burn ignition altitude). Use as
%   odeset('Events', @(t, s) altitudeEvent(t, s, yEvent)).
value = state(2) - yEvent;
isterminal = 1;
direction = -1; % only trigger when descending through yEvent
end

function [value, isterminal, direction] = touchdownEvent(~, state)
%TOUCHDOWNEVENT ODE event function that stops integration at y = 0
%   (ground contact), approaching from above.
y = state(2);
value = y;
isterminal = 1;
direction = -1; % only trigger when descending through y = 0
end

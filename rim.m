function dx = rim(t, x, m, mw, M, ang_inr, lwr, nw, g, l, T)
% RIM - ODE function for rimless wheel dynamics
%
% Inputs:
%   t        - Current time (not used)
%   x        - State vector [theta; theta_dot]
%   m, mw    - Mass of hub and spokes
%   M        - Moment of inertia
%   ang_inr  - Relative angle between spoke and vertical
%   lwr      - Distance from hub to CoM of each spoke
%   nw       - Number of spokes
%   g        - Gravitational acceleration projected on slope
%   l        - Length of each spoke
%   T        - External torque (e.g., terrain reaction)
%
% Output:
%   dx       - State derivative [theta_dot; theta_ddot]

theta = x(1);
theta_dot = x(2);

% Gravity-induced torque
% Hub contribution: m * l * g * sin(theta)
% Spokes contribution: mw * sum of all individual torque arms
spoke_torque = mw * sum(lwr .* sin(theta*ones(1,nw) - ang_inr)) * g;
hub_torque   = m * l * g * sin(theta);
P = hub_torque + spoke_torque;

% Angular acceleration (Net torque / Inertia)
theta_ddot = (-P + T) / M;

% Return derivatives
dx = [theta_dot; theta_ddot];
end

clc; clear; close all;

%% Parameters
nw = 7;               % Number of spokes
m = 10;               % Mass of hub
mw = 1;               % Mass per spoke
l = 1;                % Spoke length
g = 9.81;             % Gravity
gamma = -10*pi/180;    % Slope inclination

% Contact model parameters (stiff virtual ground spring-damper)
k_ground = 1e5;       % Ground stiffness (N/m)
d_ground = 20000000;       % Ground damping (Ns/m)

% Moment of inertia
I = m*l^2 + nw*mw*l^2;  % Approximate

% Spoke angles
theta_s = linspace(0, 2*pi - 2*pi/nw, nw);

%% Initial conditions [theta; theta_dot]
theta0 = 0;
thetadot0 = 0;
x0 = [theta0; thetadot0];

%% Simulation setup
tspan = [0 3];

% Run ODE
[t, x] = ode45(@(t,x) rimless_aug(t,x,m,mw,I,nw,l,g,gamma,k_ground,d_ground,theta_s), tspan, x0);

%% Plotting
figure;
subplot(2,1,1);
plot(t, x(:,1), 'b'); ylabel('\theta (rad)'); title('Rimless Wheel with Augmented Contact');
subplot(2,1,2);
plot(t, x(:,2), 'r'); ylabel('d\theta/dt (rad/s)'); xlabel('Time (s)');



%%
%% Spoke Animation for Augmented Rimless Wheel
y = x;
% Fixed Animation for Augmented Rimless Wheel
theta_vals = y(:,1);      % θ (angle)
thetadot_vals = y(:,2);   % θ̇ (angular velocity)
num_frames = length(theta_vals);
theta_s = linspace(0, 2*pi - 2*pi/nw, nw);  % spoke angles

% Pre-compute spoke angle offset (rotation from vertical stance)
stance_angle_offset = mod(theta_vals + pi, 2*pi);  % rotate so stance spoke is at contact

% Approx rolling arc length
arc_lengths = cumtrapz(t, abs(thetadot_vals)) * l;  % approximated arc-length
hub_base = arc_lengths;  % use as horizontal offset

% Inclined plane direction
dx = cos(gamma);
dy = sin(gamma);

% Ground line slope
xg = linspace(0, max(hub_base)*1.2, 100);
yg = tan(gamma) * xg;

% Setup figure
figure;
axis equal;
hold on;
xlabel('X'); ylabel('Y');
title('Rimless Wheel on Incline (Augmented Model)');
plot(xg, yg, 'r--', 'LineWidth', 1.5);  % inclined plane

spoke_lines = gobjects(nw, 1);
for j = 1:nw
    spoke_lines(j) = plot([0 0], [0 0], 'k', 'LineWidth', 2);
end
hub_marker = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Animation
for i = 1:10:num_frames
    theta_i = theta_vals(i);
    
    % Set stance foot at ground line (rolls forward)
    base_x = hub_base(i) * dx;
    base_y = hub_base(i) * dy;
    
    R = [cos(theta_i), -sin(theta_i); sin(theta_i), cos(theta_i)];
    
    % Hub position = stance foot + R*(-l * e_theta)
    stance_vec = R * [-l; 0];  % vertical spoke downward
    hub_pos = [base_x; base_y] - stance_vec;

    % Update spoke lines
    for j = 1:nw
        spoke_dir = [cos(theta_s(j)); sin(theta_s(j))];
        tip = R * (l * spoke_dir);
        set(spoke_lines(j), 'XData', [hub_pos(1), hub_pos(1) + tip(1)], ...
                            'YData', [hub_pos(2), hub_pos(2) + tip(2)]);
    end

    % Update hub
    set(hub_marker, 'XData', hub_pos(1), 'YData', hub_pos(2));
    
    drawnow;
end

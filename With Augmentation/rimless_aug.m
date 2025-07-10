function dx = rimless_aug(t, x, m, mw, I, nw, l, g, gamma, k_ground, d_ground, theta_s)
% RIMLESS_AUG - Augmented dynamics with continuous ground reaction forces

theta = x(1);
theta_dot = x(2);

% Wheel center position
xc = 0; yc = 0;

% Rotation matrix for hub orientation
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

% Inclination rotation
R_gamma = [cos(gamma) sin(gamma); -sin(gamma) cos(gamma)];

% Gravity vector in slope frame
g_vec = R_gamma * [0; -g];

% Initialize total torque and GRF tracking
total_torque = 0;

for i = 1:nw
    % Spoke vector in body frame
    spoke_dir = [cos(theta_s(i)); sin(theta_s(i))];
    
    % Foot position in world frame
    foot_pos = [xc; yc] + R * (l * spoke_dir);
    
    % Ground height is along inclined plane: y = tan(gamma) * x
    y_ground = foot_pos(1) * tan(gamma);
    
    % Penetration depth
    penetration = y_ground - foot_pos(2);
    
    % Contact velocity (vertical)
    foot_vel = [-l * sin(theta + theta_s(i)) * theta_dot;
                 l * cos(theta + theta_s(i)) * theta_dot];
    normal_dir = [-sin(gamma); cos(gamma)];
    v_n = dot(foot_vel, normal_dir);
    
    % Only apply contact force if penetration is positive
    if penetration > 0
        f_n = k_ground * penetration + d_ground * v_n; % Normal force
        % Clamp to non-negative (no adhesion)
        if f_n < 0, f_n = 0; end
        
        % Apply torque = r × F
        r = R * (l * spoke_dir);  % vector from center to foot
        Fg = f_n * normal_dir;    % ground reaction force
        torque = r(1) * Fg(2) - r(2) * Fg(1);  % scalar cross product
        total_torque = total_torque + torque;
    end
end

% Add gravitational torque (about center)
torque_gravity = -m * l * g * sin(theta - gamma);

% Net torque
net_torque = torque_gravity + total_torque;

% Angular acceleration
theta_ddot = net_torque / I;

% Return state derivative
dx = [theta_dot; theta_ddot];
end

<<<<<<< HEAD
%% Rimless Wheel Simulation on Inclined Plane
% Author: Rajan Prasad
% This code simulates the passive dynamics of a rimless wheel rolling down an incline
% and creates an animation with torque, trajectory, and phase portrait plots.

=======
>>>>>>> ddb89013c656419882386b8734e1b541b51e9d29
clc;
clear;
close all;

<<<<<<< HEAD
%% ------------------------- [1] Setup & Parameters -------------------------

% Inclination angle of the slope (in radians)
gamma = 10 * pi / 180;

% Rimless wheel parameters
m = 10;      % Mass of the hub (center)
l = 1;       % Length of each spoke
g = 9.81 * cos(gamma); % Effective gravity along slope
mw = 1;      % Mass per spoke
lw = 0.5;    % Distance of spoke CoM from hub
nw = 12;      % Number of spokes

% Geometry of spokes
ang_w = 0:2*pi/nw:(2*pi - 2*pi/nw);  % Angles between spokes (rad)
ang_d = 0:360/nw:(360 - 360/nw);    % Degrees between spokes

% Distance from hub to CoM of each spoke
lwr = sqrt((l^2 + lw^2) - 2*l*lw.*cosd(ang_d));

% Angle between spoke line and vector to CoM
ang_in = asind((lw .* sind(ang_d)) ./ lwr);
ang_inr = deg2rad(ang_in);  % Convert to radians

% Total mass moment of inertia
M = m * l^2 + mw * sum(lwr.^2);

%% ------------------------- [2] Initial Conditions -------------------------

a = pi;         % Initial angle (rad)
b = 0.5;        % Initial angular velocity (rad/s)
yo = [a; b];    % State vector [theta; theta_dot]

% Time simulation setup
=======
%% Specify the inclination
gamma = 10*pi/180;

%% Rimless wheel parameter definition
m = 10; % Mass of the center of the wheel or [HUB]
l = 1; % Length of the rim rods [SPOKES]
g = 9.81*cos(gamma); % Gravity including inclination effect
mw = 1; % Mass of spokes
lw = 0.5; % mass center of spokes
nw = 10; % Number of spokes

%% Wheel other parameter estimations
ang_w = 0:6.28/nw:(6.28-6.28/nw); % Angle between SPOKES
ang_d = 0:360/nw:(360-360/nw);
lwr = ((l^2+lw^2)*ones(1,nw)-2*l*lw*cosd(ang_d)).^0.5;
ang_in = asind((lw.*sind(ang_d))./lwr);
ang_inr = ang_in*pi/180;
% Mass moment of Inertia of wheel
M = m*l^2+mw*sum((lwr(1:nw)).^2);


%% Initial Condition definition
a = pi; %theta
b = 0.5; %theta dot
yo = [a;b]; % Combined initial

%% Simulation parameters definitions
>>>>>>> ddb89013c656419882386b8734e1b541b51e9d29
tsam = 0.01;
t_end = 10;
ts = 0:tsam:t_end;
ia = length(ts);
<<<<<<< HEAD

% Initialize storage
F = zeros(ia,1);     % Torque history
swi(1) = 1;          % Step change index
xc = 0;              % Wheel x-position

% Initial torque estimation
Tor = -(m*l*cos(a) + mw*sum(lwr .* cos(ang_inr + a))) * g * sin(gamma);
F(1) = Tor;

count = 2;

%% ------------------------- [3] Simulation Loop -------------------------

for i = 1:ia
    % Solve ODE for current timestep
    [T1,Y1] = ode45(@rim,[(i-1)*tsam i*tsam],yo,[],m,mw,M,ang_inr,lwr,nw,g,l,Tor);
    
    % Update torque for new state
    Tor = -(m*l*cos(Y1(end,1)) + mw*sum(lwr .* cos(ang_inr + Y1(end,1)))) * g * sin(gamma);
    F(i) = Tor;
    
    t(i) = T1(end);
    y(i,:) = Y1(end,:);
    yo = Y1(end,:)';  % Set initial condition for next step
    
    % Spoke impact check and state reset
    if Y1(end,1) >= (pi + pi/nw)
        xc = xc - l*sin(yo(1)) - l*sin(yo(1) + ang_w(end) - pi);
        xr(i,1) = xc;
        
        swi(count) = i;
        count = count + 1;
        
        % Reset angle and angular velocity post-impact
        yo = [Y1(end,1) - 2*pi/nw;
              Y1(end,2) * cos(2*pi/nw)];
        y(i,:) = yo';
    else
=======
F = zeros(length(ts),1);

% Inclination
Rot2 = [-sin(gamma) cos(gamma)];
% Gravity Torque Exerted
Tor = -(m*l*cos(a)+mw*sum(lwr.*cos(ang_inr+(a)*ones(1,nw))))*g*sin(gamma); %N/m
% Other parameter initialization
F(1) = Tor;
count = 2;
swi(1) = 1;
xc = 0;

%% Actual simulation begins here
for i = 1:ia
    % ODE Model of the RIMLESS WHEEL {Saved as rim}
    [T1,Y1] = ode45(@rim,[(i-1)*tsam i*tsam],yo,[],m,mw,M,ang_inr,lwr,nw,g,l,Tor);
    % Estimating the torque based on kinematics for next step and storing it
    Tor = -(m*l*cos(Y1(end,1))+mw*sum(lwr.*cos(ang_inr+(Y1(end,1))*ones(1,nw))))*g*sin(gamma);
    F(i,1) = Tor;
    t(1,i) = T1(end);
    y(i,1:2) = Y1(end,1:2);
    yo = Y1(end,1:2); % Updating initial condition for next step

% Rearranging angles when rolling
    if Y1(end,1)>= (pi+pi/nw)
        xc = xc-l*sin(yo(1))-l*sin(yo(1)+ang_w(nw)-pi);
        xr(i,1) = xc;
        %         disp(yo);
        swi(count) = i;
        count = count+1;
        yo = [Y1(end,1)-(2*pi/nw);Y1(end,2)*cos(2*pi/nw)];
        y(i,1:2) = yo;
        %break;
    else
        y(i,1:2) = Y1(end,1:2);
>>>>>>> ddb89013c656419882386b8734e1b541b51e9d29
        xr(i,1) = xc;
    end
end

<<<<<<< HEAD
%% ------------------------- [4] Post Processing -------------------------

% Hub position (straight projection)
xp_s = xr - l * sin(y(:,1));
yp_s = -l * cos(y(:,1));

% Compute spoke endpoints (straight projection)
n = length(t);
up_ang = y(:,1)*ones(1,nw) + ones(n,1)*ang_w - pi;
xpw_s = xp_s * ones(1,nw) - l * sin(up_ang);
ypw_s = yp_s * ones(1,nw) - l * cos(up_ang);

% Rotation matrix to incline the plane
Rot = [cos(gamma) sin(gamma); -sin(gamma) cos(gamma)];

% Rotate the system to match inclined plane view
for q1 = 1:n
    for q2 = 1:nw
        New = Rot * [xpw_s(q1,q2); ypw_s(q1,q2)];
=======
%% Post Processing Data
% Straight axis
xp_s = xr-l*sin(y(:,1));
yp_s = -l*cos(y(:,1));
n = length(t);
up_ang = y(:,1)*ones(1,nw)+ones(n,1)*ang_w-pi*ones(n,nw);
xpw_s = xp_s*(ones(1,nw))-l.*sin(up_ang);
ypw_s = yp_s*(ones(1,nw))-l.*cos(up_ang);

%_____________________Using axis Rotataion_____________________________

Rot = [cos(gamma) sin(gamma);-sin(gamma) cos(gamma)];
for q1 = 1:n
    for q2 = 1:nw
        New = Rot*[xpw_s(q1,q2);ypw_s(q1,q2)];
>>>>>>> ddb89013c656419882386b8734e1b541b51e9d29
        xpw(q1,q2) = New(1);
        ypw(q1,q2) = New(2);
    end
end

<<<<<<< HEAD
% Rotate hub position
X = [xp_s yp_s]';
X1 = Rot * X;
xp = X1(1,:)';
yp = X1(2,:)';

% Inclined ground boundary
L1 = max(max(xpw)) + 5;
L2 = Rot * [L1; 0];

%% ------------------------- [5] Energy and Torque -------------------------

% Torque at hub (estimated)
T = ((m * y(:,2).^2) / l + m*g*sin(y(:,1))) * l;

% Energy computations
P = m * g * yp;                         % Potential energy
KE = 0.5 * m * l^2 * y(:,2).^2;         % Kinetic energy
TE = KE - P;                            % Total energy

%% ------------------------- [6] Visualization and Animation -------------------------

figure;
set(gcf, 'position', [10, 10, 2100, 2100]);

% Axis boundaries
axisp_s = [min(min(xpw_s)) max(max(xpw_s)) min(min(ypw_s)) max(max(ypw_s))];
axisp = [min(min(xpw)) max(max(xpw)) min(min(ypw), L2(2)) max(max(ypw)) + 1];
axist = [0 t(end) min(F) max(F)];
ax4 = [min(y(:,1)) max(y(:,1)) min(y(:,2)) max(y(:,2))];

% Frame collection
f_count = 1;
k = 1;

while (k < n)
    k2 = k + 1;
    
    % Determine active spoke
    a1 = find(swi < k2, 1, 'last');
    a2 = mod(a1, nw);
    if a2 == 0, a2 = nw; end
    
    % Track specific spoke
    ntrw = 1;
    a3 = mod(a2 + ntrw - 1, nw);
    if a3 == 0, a3 = nw; end
    
    A(f_count,:) = [xpw(k,a3), ypw(k,a3)];
    A_s(f_count,:) = [xpw_s(k,a3), ypw_s(k,a3)];

    % ---------- Subplot 1: Inclined View ----------
    subplot(2,2,1);
    hold on;
    fill([0 L2(1) 0], [0 L2(2) L2(2)], 'r'); % Ground
    plot(xp(k), yp(k), 'or', 'MarkerSize', 20, 'MarkerFaceColor', [1 0.1 .3]);
    title(sprintf('Passive @Time = %.1f s', t(k)));
    plot(xp(1:k), yp(1:k), '--');
    plot(A(1:f_count,1), A(1:f_count,2), '--g');
    for k1 = 1:nw
        ky = mod(k1 + a2 - 1, nw);
        if ky == 0, ky = nw; end
        plot([xp(k) xpw(k,ky)], [yp(k) ypw(k,ky)], 'LineWidth', 2);
    end
    
    % ---------- Subplot 2: Straight Projection ----------
    subplot(2,2,2);
    hold on;
    line([0 max(max(xpw_s)) + 2], [0 0]);
    plot(xp_s(k), yp_s(k), 'or', 'MarkerSize', 20, 'MarkerFaceColor', [1 0.1 .3]);
    title(sprintf('Active @Time = %.1f s', t(k)));
    plot(xp_s(1:k), yp_s(1:k), '--');
    plot(A_s(1:f_count,1), A_s(1:f_count,2), '--g');
    for k1 = 1:nw
        ky = mod(k1 + a2 - 1, nw);
        if ky == 0, ky = nw; end
        plot([xp_s(k) xpw_s(k,ky)], [yp_s(k) ypw_s(k,ky)], 'LineWidth', 2);
    end

    % ---------- Subplot 3: Torque Plot ----------
    subplot(2,2,3);
    hold on; grid on;
    xlim([0 t(end)]);
    plot(t(1:k), F(1:k), '-r');
    plot(t(k), F(k), 'or');
    xlabel('Time (s)'); ylabel('Torque (Nm)');
    title('Torque required for the motion');

    % ---------- Subplot 4: Phase Portrait ----------
    subplot(2,2,4);
    hold on;
    plot(y(1:k,1), y(1:k,2), '*r', 'MarkerSize', 2);
    xlabel('\theta (rad)'); ylabel('d\theta/dt (rad/s)');
    title('Phase Portrait');

    % ---------- Save Frame ----------
    frame = getframe(gcf);
    im(f_count) = frame;
    im2{f_count} = frame2im(frame);
    
    if k < (n - 1)
        clf;
    end
    
    k = k + 10;
    f_count = f_count + 1;
end

%% ------------------------- [7] Video Export -------------------------
v = VideoWriter('RimLess Wheel.mp4', 'MPEG-4');
open(v);
writeVideo(v, im);
close(v);


%% ------------------------- [9] GIF Export -------------------------
filename = 'rimless_wheel.gif'; % Output file name
delay = 0.05;                   % Delay time between frames (in seconds)

for idx = 1:f_count-1
    [A_map, map] = rgb2ind(im2{idx}, 256); % Convert RGB to indexed image
    
    if idx == 1
        % Create the GIF file
        imwrite(A_map, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', delay);
    else
        % Append to the existing GIF
        imwrite(A_map, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
    end
end
=======
X = [xp_s yp_s]';
X1 = Rot*X;
xp = X1(1,:)';
yp = X1(2,:)';


L1 = max(max(xpw))+5;
L2 = Rot*[L1;0];

%% Torque at center
T = ((m*(y(:,2)).^2)/l+m*g*sin(y(:,1)))*l;

%% Enegry estimation
P = m*g*yp;
KE = 0.5.*m*l^2*(y(:,2)).^2;
TE = KE-P;

%% Result plot
figure;
l0 = 10;
b0 = 10;
width = 2100;
height = 2100;
set(gcf,'position',[l0,b0,width,height]);


if max(yp)<0
    my = 0.1;
else
    my = max(yp);
end
axisp_s = [min(min(xpw_s)) max(max(xpw_s)) min(min(ypw_s)) max(max(ypw_s))];
axisp = [min(min(xpw)) max(max(xpw)) min(min(min(ypw)),L2(2)) max(max(ypw))+1];

%--------------------------------
axist = [0 t(end) min(F) (max(F))];
axispe = [0 t(end) min(TE)*1.1 (max(TE))];
ax4 = [min(y(:,1))-0.1 max(y(:,1))+0.1 min(y(:,2))-0.1 max(y(:,2))+0.1];

f_count = 1;
k = 1;
k2 = 1;
while (k<n)
    %-----------Pendulum plot---------------
    k2 = k+1;
    a1 = find(swi<k2);
    a1 = a1(end);
    a2 = rem(a1,nw);

    if a2 == 0
        a2 = nw;
    end
    %-----Track a wheel----------------------
    ntrw = 6;% tracker wheel number
    a3 = a2+ntrw;
    if a3>nw
        a3 = a3-nw;
    end
    A(f_count,1:2) = [xpw(k,a3);ypw(k,a3)];
    A_s(f_count,1:2) = [xpw_s(k,a3);ypw_s(k,a3)];
    %--------------------------------
    % Inclined plane plot
    subplot(2,2,1)
    hold on;
    %     axis(axisp);
    fill([0 L2(1) 0],[0 L2(2) L2(2)],'r');
    plot(xp(k),yp(k),'or','MarkerSize',20,'MarkerFaceColor',[1 0.1 .3]);
    str = sprintf('Passive @Time = %.1f s',t(k));
    title(str);
    plot(xp(1:k),yp(1:k),'--');
    hold all;
    %------------tracked wheel------------
    plot(A(1:f_count,1),A(1:f_count,2),'--g');
    %-------------------------------------------
    for k1 = 1:nw
        ky = k1+a2-1;
        if ky>nw
            ky = ky-nw;
        end
        hold on;
        plot([xp(k) xpw(k,ky)],[yp(k) ypw(k,ky)],'LineWidth',2);
    end


    % Straight projection
    %-----------------------Straight-------------------
    subplot(2,2,2)
    hold on;
    %     axis(axisp);
    line([0 max(max(xpw_s))+2],[0 0]);
    plot(xp_s(k),yp_s(k),'or','MarkerSize',20,'MarkerFaceColor',[1 0.1 .3]);
    str = sprintf('Active @Time = %.1f s',t(k));
    title(str);
    plot(xp_s(1:k),yp_s(1:k),'--');
    hold all;

    %------------tracked wheel------------
    plot(A_s(1:f_count,1),A_s(1:f_count,2),'--g');
    %-------------------------------------------
    for k1 = 1:nw
        ky = k1+a2-1;
        if ky>nw
            ky = ky-nw;
        end
        hold on;
        plot([xp_s(k) xpw_s(k,ky)],[yp_s(k) ypw_s(k,ky)],'LineWidth',2);
    end

    %--------Torque Plot-------------
    subplot(2,2,3)
    grid on;
    %     axis(axist);
    xlim([0 t(end)])
    hold on;
    plot(t(1:k),F(1:k),'-r');
    plot(t(k),F(k),'o-');
    xlabel('Time(s)');
    ylabel('Torque(Nm)');
    title('Torque required for the motion');

    %--------System total energy Plot
    %     subplot(2,3,3)
    %     axis(axispe);
    %     hold on;
    %     plot(t(1:k),TE(1:k),'-<');
    %     hold on;
    %     xlabel('Time(s)');
    %     ylabel('T.E.');
    %     title('Energy');

    %--------System phase portrait-------------
    subplot(2,2,4)
    %     axis(ax4);
    hold on;
    plot(y(1:k,1),y(1:k,2),'*r','MarkerSize',2);
    hold on;
    %     plot(y(k,1),y(k,2),'or');
    xlabel('\theta (rad)');
    ylabel('d\theta/dt (rad/s)');
    title('Phase Portrait of both case');



    drawnow;
    %----------frame for video/gif file----------------
    frame  =  getframe(gcf);
    im(f_count) = frame;
    im2{f_count}  =  frame2im(frame);
    %------------------------------------------------------------
    if k<(n-1)
        clf;
    else
        drawnow;  % pause(0.4);
    end
    % Upadate counter
    k = k+10;
    f_count = f_count+1;
end

v  =  VideoWriter('RimLess Wheel.mp4','MPEG-4');
open(v);
writeVideo(v,im);
close(v);
>>>>>>> ddb89013c656419882386b8734e1b541b51e9d29

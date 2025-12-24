%% Workspace Calculation of Right Leg Using FK:

clc; close all; clear;

% measured lengths
l1 = 3.1;    % hip length
l2 = 4.5;   % knee length
l3 = 3.0;   % bend length
l4 = 9;    % ankle length

% angle ranges
theta1_min = -45;  theta1_max = 45;
theta2_min = -90;  theta2_max = 90;
theta4_min = -90;  theta4_max = 90;

theta3 = pi/4; % constant angle 

valid_pts = [];  % will store valid [x,y,z]

% theta values
th1 = linspace(theta1_min, theta1_max, 30);
th2 = linspace(theta2_min, theta2_max, 30);
th4 = linspace(theta4_min, theta4_max, 30);

for th1i = 1:length(th1)
    for th2i = 1:length(th2)
        for th4i = 1:length(th4)
            theta1 = deg2rad(th1(th1i));
            theta2 = deg2rad(th2(th2i));
            theta4 = deg2rad(th4(th4i));
            
            x = cos(theta1)*(l1 + l4*cos(theta3 - theta2 + theta4) + l2*cos(theta2) + l3*cos(theta2 - theta3));
            y = sin(theta1)*(l1 + l4*cos(theta3 - theta2 + theta4) + l2*cos(theta2) + l3*cos(theta2 - theta3));
            z = l2*sin(theta2) - l4*sin(theta3 - theta2 + theta4) + l3*sin(theta2 - theta3);

            valid_pts(end+1, :) = [x, y, z];                 
        end
    end
end

sz = size(valid_pts)

figure;
scatter3(valid_pts(:,1), valid_pts(:,2), valid_pts(:,3), 4, 'b', 'filled');
hold on;
scatter3(0, 0, 0, 60, 'y', 'filled'); % black dot at leg origin
text(0, 0, 0, '  Leg Base', 'FontWeight','bold', 'Color','w');
xlabel('x'); ylabel('y'); zlabel('z');
title('Right leg workspace using FK');
xlim([-30 30]);
ylim([-30 30]);
grid on;

%% Workspace Calculation of Left Leg Using FK:

clc; close all; clear;

% measured lengths
l1 = 3.1;    % hip length
l2 = 4.5;   % knee length
l3 = 3.0;   % bend length
l4 = 9;    % ankle length

% angle ranges
theta1_min = -45;  theta1_max = 45;
theta2_min = -90;  theta2_max = 90;
theta4_min = -90;  theta4_max = 90;

theta3 = -pi/4; % constant angle

valid_pts = [];  % will store valid [x,y,z]

th1 = linspace(theta1_min, theta1_max, 30);
th2 = linspace(theta2_min, theta2_max, 30);
th4 = linspace(theta4_min, theta4_max, 30);

for th1i = 1:length(th1)
    for th2i = 1:length(th2)
        for th4i = 1:length(th4)
            theta1 = deg2rad(th1(th1i));
            theta2 = deg2rad(th2(th2i));
            theta4 = deg2rad(th4(th4i));
            
            x = cos(theta1)*(l1 + l4*cos(theta3 - theta2 + theta4) + l2*cos(theta2) + l3*cos(theta2 - theta3));
            y = sin(theta1)*(l1 + l4*cos(theta3 - theta2 + theta4) + l2*cos(theta2) + l3*cos(theta2 - theta3));
            z = -(l2*sin(theta2) - l4*sin(theta3 - theta2 + theta4) + l3*sin(theta2 - theta3));
            valid_pts(end+1, :) = [x, y, z];                 
        end
    end
end

sz = size(valid_pts)

figure;
scatter3(valid_pts(:,1), valid_pts(:,2), valid_pts(:,3), 4, 'b', 'filled');
hold on;
scatter3(0, 0, 0, 60, 'y', 'filled'); % black dot at leg origin
text(0, 0, 0, '  Leg Base', 'FontWeight','bold', 'Color','w');
xlabel('x'); ylabel('y'); zlabel('z');
title('Left leg workspace using FK');
xlim([-30 30]);
ylim([-30 30]);
grid on;

%% Workspace Calculation of Right Leg Using IK:
% asin sols are chosen, but we also have 4 possibilities of acos sols

clc; close all; clear;

% measured lengths
l1 = 3.1;    % hip length
l2 = 4.5;   % knee length
l3 = 3.0;   % bend length
l4 = 9;    % ankle length

% angle ranges
theta1_min = -45;  theta1_max = 45;
theta2_min = -90;  theta2_max = 90;
theta4_min = -90;  theta4_max = 90;

theta3 = -pi/4; % constant angle

% Define sampling region in foot endpoint space (cm)
x_low = -30;
x_high = 30;
x_range = linspace(x_low, x_high, 50); 
y_low = -30;
y_high = 30;
y_range = linspace(y_low, y_high, 50);
z_low = -20;
z_high = 20;
z_range = linspace(z_low, z_high, 50);

valid_pts = [];  % will store valid [x,y,z]

for xi = 1:length(x_range)
    for yi = 1:length(y_range)
        for zi = 1:length(z_range)
            x = x_range(xi);
            y = y_range(yi);
            z = z_range(zi);
            
            theta1 = atan2(y, x);   % hip joint angle
            d1 = rad2deg(theta1);
            
            if isreal(theta1) && d1 >= theta1_min && d1 <= theta1_max
                C = (x*cos(theta1) - l1 + y*sin(theta1))^2 + z^2;
                D = (C-l2^2-l3^2-l4^2-2*l2*l3*cos(theta3))/(2*l4);
                A = l2*cos(theta3) + l3;
                B = l2*sin(theta3);
                phi = atan2(A, B);   
                a = A/sin(phi);
                theta4 = phi - asin(D/a);
                d4 = rad2deg(theta4);
                
                if  isreal(theta4) && d4 >= theta4_min && d4 <= theta4_max

                    alp = l2 + l4*cos(theta3+theta4) + l3*cos(theta3);
                    beta = l4*sin(theta3+theta4) + l3*sin(theta3);
                    phi2 = atan2(beta, alp);    % sin sol
                    a2 = beta/sin(phi2); 
                    theta2 = phi2 + asin(z/a2);                
                    d2 = rad2deg(theta2);
                    
                    if isreal(theta2) && d2 >= theta2_min && d2 <= theta2_max
                        valid_pts(end+1, :) = [x, y, z];
                    end
                end
            end
        end
    end
end

sz = size(valid_pts)
% cospt = valid_pts;

figure;
scatter3(valid_pts(:,1), valid_pts(:,2), valid_pts(:,3), 8, 'm', 'filled');
hold on;
scatter3(0, 0, 0, 60, 'y', 'filled'); % black dot at leg origin
text(0, 0, 0, '  Leg Base', 'FontWeight','bold', 'Color','w');
xlabel('x'); ylabel('y'); zlabel('z');
title('Right leg workspace using IK');
xlim([-30 30]);
ylim([-30 30]);
grid on;

%% Workspace Calculation of Left Leg Using IK:
% asin sols are chosen, but we also have 4 possibilities of acos sols

clc; close all; clear;

% measured lengths
l1 = 3.1;    % hip length
l2 = 4.5;   % knee length
l3 = 3.0;   % bend length
l4 = 9;    % ankle length

% angle ranges
theta1_min = -45;  theta1_max = 45;
theta2_min = -90;  theta2_max = 90;
theta4_min = -90;  theta4_max = 90;

theta3 = -pi/4; % constant angle

% Define sampling region in foot endpoint space (cm)
% lower fidelity
x_low = -30;
x_high = 30;
x_range = linspace(x_low, x_high, 50);
y_low = -30;
y_high = 30;
y_range = linspace(y_low, y_high, 50);
z_low = -20;
z_high = 20;
z_range = linspace(z_low, z_high, 50);

valid_pts = [];  % will store valid [x,y,z]

for xi = 1:length(x_range)
    for yi = 1:length(y_range)
        for zi = 1:length(z_range)
            x = x_range(xi);
            y = y_range(yi);
            z = z_range(zi);
            
            theta1 = atan2(y, x);   % hip joint angle
            d1 = rad2deg(theta1);
            
            if isreal(theta1) && d1 >= theta1_min && d1 <= theta1_max
                C = (x*cos(theta1) - l1 + y*sin(theta1))^2 + z^2;
                D = (C-l2^2-l3^2-l4^2-2*l2*l3*cos(theta3))/(2*l4);
                A = l2*cos(theta3) + l3;
                B = l2*sin(theta3);
                phi = atan2(A, B);    % sin sol
                a = A/sin(phi);
                theta4 = phi - asin(D/a);
                % phi = atan2(B, A);    % cos sol
                % a = B/sin(phi);
                % theta4 = -acos(D/a) - phi;
                
                % paper eq
                % theta3 = asin( ( (cos(theta1)*x + sin(theta1)*y - l1)^2 + z^2 - l2^2 - l3^2 ) / (2*l2*l3) ) - pi/2;

                d4 = rad2deg(theta4);
                
                if  isreal(theta4) && d4 >= theta4_min && d4 <= theta4_max

                    alp = l2 + l4*cos(theta3+theta4) + l3*cos(theta3);
                    beta = l4*sin(theta3+theta4) + l3*sin(theta3);
                    phi2 = atan2(beta, alp);    % sin sol
                    a2 = beta/sin(phi2); 
                    theta2 = phi2 + asin(-z/a2);
                    % phi2 = atan2(alp, beta);    % cos sol
                    % a2 = alp/sin(phi2);
                    % theta2 = acos(z/a2) - phi2;
                 
                    % paper eq
                    % theta2 = atan2(l3*sin(theta3), l3*cos(theta3) + l2) + acos( z / sqrt( (l3*sin(theta3))^2 + (l3*cos(theta3) + l2)^2 ) );

                    d2 = rad2deg(theta2);
                    
                    if isreal(theta2) && d2 >= theta2_min && d2 <= theta2_max
                        valid_pts(end+1, :) = [x, y, z];
                    end
                end
            end
        end
    end
end

sz = size(valid_pts)
% cospt = valid_pts;

figure;
scatter3(valid_pts(:,1), valid_pts(:,2), valid_pts(:,3), 4, 'b', 'filled');
hold on;
scatter3(0, 0, 0, 60, 'y', 'filled'); % black dot at leg origin
text(0, 0, 0, '  Leg Base', 'FontWeight','bold', 'Color','k');
xlabel('x'); ylabel('y'); zlabel('z');
title('Left leg workspace using IK');
xlim([-30 30]);
ylim([-30 30]);
grid on;

%% If legacy, then delete

% % === Check whether each Bezier point exists in valid_pts ===
% num_cospt = size(cospt, 1);
% 
% for i = 1:num_cospt
%     pt = cospt(i, :);   % 1x3 point
% 
%     % check if this 1x3 row exists in valid_pts
%     isPresent = ismember(valid_pts, pt, 'rows');
% 
%     if ~any(isPresent)
%         fprintf("xyz(%d,:) = [%.3f  %.3f  %.3f] is NOT in valid_pts\n", ...
%             i, pt(1), pt(2), pt(3));
%     else
%         continue
%     end
% end

%% Bezier curve generation

X = 16; % Lateral distance from the hip
S = -5; % Verical distance of the stance line 
A = 6; % Vertical lift from the stance line
T = 14; % Length of the stance line

% Bezier control points
P1 = [-T/2, S];
P2 = [0, S + 2*A];
P3 = [T/2, S];

NUM_DATA_POINTS = 20;
half = NUM_DATA_POINTS / 2;

t = linspace(0, 1, half);
xyz = zeros(NUM_DATA_POINTS, 3);

% Swing phase (curved)
for i = 1:half
    xyz(i, 1) = X;
    xyz(i, 2) = (1 - t(i))^2 * P1(1) + 2*(1 - t(i))*t(i)*P2(1) + t(i)^2 * P3(1);
    xyz(i, 3) = (1 - t(i))^2 * P1(2) + 2*(1 - t(i))*t(i)*P2(2) + t(i)^2 * P3(2);
end

% Stance phase (straight line)
y_stance = linspace(T/2, -T/2, half);
for i = 1:half
    xyz(half + i, 1) = X;
    xyz(half + i, 2) = y_stance(i);
    xyz(half + i, 3) = S;
end

% Display trajectory on workspace
hold on;
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'r-', 'LineWidth', 2);
scatter3(xyz(:,1), xyz(:,2), xyz(:,3), 20, 'r', 'filled');
legend('Workspace', 'Leg Base', 'Trajectory', 'Location', 'best');

%% Transform workspace and trajectory (FRONT RIGHT LEG)

% Rotation matrix
theta = deg2rad(45);  % rotation angle
Rz = [cos(theta) -sin(theta) 0;
      sin(theta)  cos(theta) 0;
      0           0          1];
offsets = [5; 10; 0];       % distance of front right leg from robot base

T_leg_to_base = [Rz, offsets; 0 0 0 1];  % Homogeneous transformation

% Transform workspace
valid_pts_h = [valid_pts, ones(size(valid_pts,1),1)]';  % make homogeneous
valid_pts_global = (T_leg_to_base * valid_pts_h)';       % transform
valid_pts_global = valid_pts_global(:,1:3);              % drop homogeneous coord

% Transform trajectory
theta_t = -1*theta; % to make the trajectory parallel with the robot
Rz_t = [cos(theta_t) -sin(theta_t) 0;
        sin(theta_t)  cos(theta_t) 0;
        0           0          1];
t_t = [0; 0; 0];       % no translation post rotation

T_t = [Rz_t, t_t; 0 0 0 1];  % Homogeneous transformation

offsets_t = repmat([-2 5 0], NUM_DATA_POINTS, 1);
xyz_h = [xyz + offsets_t, ones(size(xyz,1),1)]';
xyz_global = (T_t * T_leg_to_base * xyz_h)';
xyz_global = xyz_global(:,1:3);
xyz_global(:,1) = xyz_global(:,1);
xyz_global(:,2) = xyz_global(:,2);

% Visualize
figure;
scatter3(valid_pts_global(:,1), valid_pts_global(:,2), valid_pts_global(:,3), 25, 'b', 'filled');
hold on;
scatter3(5, 10, 0, 60, 'y', 'filled');
text(5, 10, 0, '  Leg Base', 'FontWeight','bold', 'Color','k');
hold on;
scatter3(0, 0, 0, 60, 'g', 'filled');
text(0, 0, 0, '  Robot Base', 'FontWeight','bold', 'Color','k');
hold on;
plot3(xyz_global(:,1), xyz_global(:,2), xyz_global(:,3), 'r-', 'LineWidth', 2);
scatter3(xyz_global(:,1), xyz_global(:,2), xyz_global(:,3), 20, 'r', 'filled');

xlabel('X (base frame)'); ylabel('Y (base frame)'); zlabel('Z (base frame)');
title('Workspace and Trajectory in Robot Base Frame');

% isometric view
% xlim([-30 30]);
% ylim([-30 30]);
% zlim([-12 12]);

xlim([-10 45]);
ylim([-10 45]);
zlim([-15 15]);
grid on;
view(45,25);

%% LEGACY
% % Using the same T_total as above:
% T_total = T_leg_to_base;
% R = T_total(1:3,1:3);         % 3x3 rotation
% t = T_total(1:3,4)';         % 1x3 translation row
% 
% % vectorized: p_leg = R' * (p_global - t) for all rows
% xyz_leg = (R' * (xyz_global' - t'))';   % Nx3
% 
% % Overlay trajectory on workspace
% hold on;
% plot3(xyz_leg(:,1), xyz_leg(:,2), xyz_leg(:,3), 'r-', 'LineWidth', 2);
% scatter3(xyz_leg(:,1), xyz_leg(:,2), xyz_leg(:,3), 20, 'r', 'filled');
% legend('Workspace', 'Leg Base', 'Trajectory', 'Location', 'best');
% 
% xyz_le = xyz;
% 
% for row = 1:size(xyz, 1)
%     xyz_le(row, 1) = 5*cos(120) - 5*cos(60) - 10*sin(60) + 10*sin(120) + xyz(row, 1)*cos(60) + y_offset*sin(60) + xyz(row, 2)*sin(60);
%     xyz_le(row, 2) = 10*cos(120) - 10*cos(60) + 5*sin(60) - 5*sin(120) + x_offset*cos(60) + xyz(row, 2)*cos(60) - xyz(row, 1)*sin(60);
%     xyz_le(row, 3) = xyz(row, 3);
% end
% 
% % Overlay trajectory on workspace
% hold on;
% plot3(xyz_le(:,1), xyz_le(:,2), xyz_le(:,3), 'g-', 'LineWidth', 2);
% scatter3(xyz_le(:,1), xyz_le(:,2), xyz_le(:,3), 20, 'r', 'filled');
% legend('Workspace', 'Leg Base', 'Trajectory', 'Location', 'best');

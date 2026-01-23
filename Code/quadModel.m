%% Modeling TQuad Robot
%{ 
This script details the following: 
* Model of a1 single leg on each side: right and left
* 
%}

%% MODELING RIGHT LEG:

clc; clear; close all;

% measured lengths
L1 = 0.02845;    % hip length
L2 = 0.05439;   % knee length0
L3 = 0.02637;   % knee length1
L4 = 0.09265;    % ankle length

% lengths from the GitHub repository for THex
% L1 = 0.029;   % hip length
% L2 = 0.057;   % knee length
% L3 = 0.141;   % ankle length

% setting angles here results in some end effector position
t1 = deg2rad(30);  
t2 = deg2rad(54); 
tfixed = deg2rad(45); % fixed angle
t4 = deg2rad(90);

% DH Parameters for RIGHT LEG
% Taking twist for 2nd link to be pi and -pi is equivalent
dhparams = [L1     pi/2   0    t1;      % Link 1 (hip)
            L2     pi     0    t2;      % Link 2 (knee)
            L3     0      0    tfixed;  % Link 3 (bend)
            L4     0      0    t4];     % Link 4 (ankle)

leg = rigidBodyTree; % making leg structure

% assigning frames, base is hip_motor
knee = rigidBody('knee_motor');
bend = rigidBody('bend');
ankle = rigidBody('ankle_motor');
foot = rigidBody('foot');

kneejnt = rigidBodyJoint('kneejnt','revolute');
bendjnt = rigidBodyJoint('bendjnt', 'revolute');
anklejnt = rigidBodyJoint('anklejnt','revolute');
footjnt = rigidBodyJoint('footjnt','revolute');

setFixedTransform(kneejnt,dhparams(1,:),'dh'); % using DH to set the frame on the body
setFixedTransform(bendjnt,dhparams(2,:),'dh');
setFixedTransform(anklejnt,dhparams(3,:),'dh');
setFixedTransform(footjnt,dhparams(4,:),'dh');

knee.Joint = kneejnt;
bend.Joint = bendjnt;
ankle.Joint = anklejnt;
foot.Joint = footjnt;

addBody(leg,knee,'base')
addBody(leg,bend,'knee_motor')
addBody(leg,ankle,'bend')
addBody(leg,foot,'ankle_motor')

showdetails(leg)  % shows rigidbodytree with properties
config = homeConfiguration(leg); % making config to change thetas

config(1).JointPosition = t1;   % hip joint angle
config(2).JointPosition = t2;   % knee joint angle
config(3).JointPosition = tfixed;   % bend joint angle
config(4).JointPosition = t4;   % ankle joint angle

figure('Units','normalized','Position',[0.05 0.05 0.7 0.7]);
ax = axes('Parent', gcf);
hold(ax, 'on');
grid(ax, 'on');
xlabel(ax, 'x (m)'); 
ylabel(ax, 'y (m)'); 
zlabel(ax, 'z (m)');            
view(ax, 135, 25);
axis(ax, 'equal');
rotate3d on;

show(leg, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
title(ax, '3DOF Leg in Configured Pose');           
drawnow;

linkLines = findobj(ax, 'Type', 'Line');
set(linkLines, 'LineWidth', 5, 'Color', 'y');  % yellow, thick links           
%% Kinematics test for right leg

% Given a1 target end effector position, find joint angles to reach it
% Note that asin solution is used here

clc; clear; close all;

% garbage values
t1 = deg2rad(0);  
t2 = deg2rad(54); 
tfixed = deg2rad(45);  
t4 = deg2rad(90);

% measured lengths
L1 = 0.031;    % hip length
L2 = 0.045;   % knee length0
L3 = 0.030;   % knee length1
L4 = 0.09;    % ankle length

% angle ranges
theta1_min = -45;  theta1_max = 45;
theta2_min = -90;  theta2_max = 90;
theta4_min = -90;  theta4_max = 90;

dhparams = [L1     pi/2   0    t1;      % Link 1 (hip)
            L2     pi     0    t2;      % Link 2 (knee)
            L3     0      0    tfixed;  % Link 3 (bend)
            L4     0      0    t4];     % Link 4 (ankle)

leg = rigidBodyTree; % making leg structure

% assigning frames, base is hip_motor
knee = rigidBody('knee_motor');
bend = rigidBody('bend');
ankle = rigidBody('ankle_motor');
foot = rigidBody('foot');

kneejnt = rigidBodyJoint('kneejnt','revolute');
bendjnt = rigidBodyJoint('bendjnt', 'revolute');
anklejnt = rigidBodyJoint('anklejnt','revolute');
footjnt = rigidBodyJoint('footjnt','revolute');

setFixedTransform(kneejnt,dhparams(1,:),'dh'); % using DH to set the frame on the body
setFixedTransform(bendjnt,dhparams(2,:),'dh');
setFixedTransform(anklejnt,dhparams(3,:),'dh');
setFixedTransform(footjnt,dhparams(4,:),'dh');

knee.Joint = kneejnt;
bend.Joint = bendjnt;
ankle.Joint = anklejnt;
foot.Joint = footjnt;

addBody(leg,knee,'base')
addBody(leg,bend,'knee_motor')
addBody(leg,ankle,'bend')
addBody(leg,foot,'ankle_motor')

% target point
x = 0.12;
y = 0.0;
z = -0.06;

theta3 = pi/4; % constant angle

theta1 = atan2(y, x); 
d1 = rad2deg(theta1)

if isreal(theta1) && d1 >= theta1_min && d1 <= theta1_max 

    LHS = ((x*cos(theta1) - L1 + y*sin(theta1))^2 + z^2 -L2^2 - L3^2 - L4^2 - 2*L2*L3*cos(theta3))/(2*L4);
    A1 = L2*cos(theta3) + L3;
    B1 = L2*sin(theta3);
    phi1 = atan2(A1, B1);
    a1 = A1/sin(phi1);
    theta4 = phi1 - asin(LHS/a1);
    d4 = rad2deg(theta4)

    if isreal(theta4) && d4 >= theta4_min && d4 <= theta4_max
        A2 = L2 + L4*cos(theta3+theta4) + L3*cos(theta3);
        B2 = L4*sin(theta3+theta4) + L3*sin(theta3);
        phi2 = atan2(B2, A2);
        a2 = B2/sin(phi2); 
        theta2 = asin(z/a2) + phi2;
        d2 = rad2deg(theta2)

        if isreal(theta2) && d2 >= theta2_min && d2 <= theta2_max
            config = homeConfiguration(leg); % making config to change thetas    
            config(1).JointPosition = theta1;   % hip joint angle
            config(2).JointPosition = theta2;   % knee joint angle
            config(3).JointPosition = theta3;   % ankle joint angle
            config(4).JointPosition = theta4;   % ankle joint angle
            
            
            figure('Units','normalized','Position',[0.05 0.05 0.7 0.7]);
            ax = axes('Parent', gcf);
            % ax.Color = [0 0 0];
            hold(ax, 'on');
            grid(ax, 'on');
            xlabel(ax, 'x (m)'); 
            ylabel(ax, 'y (m)'); 
            zlabel(ax, 'z (m)');            
            view(ax, 135, 25);
            axis(ax, 'equal');
            rotate3d on;
            
            % draw leg
            show(leg, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
            title(ax, '3DOF Leg in Configured Pose');           
            drawnow;
            
            linkLines = findobj(ax, 'Type', 'Line');
            set(linkLines, 'LineWidth', 5, 'Color', 'y');  % yellow, thick links
            
            % draw target
            hold on
            plot3(x, y, z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); 
            text(x, y, z, '  Target', 'Color', 'r', 'FontWeight', 'bold');
            rotate3d on;
        end
    end
end
%% Bezier curve generation and rotation for front right leg

% Generate Bezier trajectory (meters)

% ORIGINAL
% X = 0.16; % far out from leg
% S = -0.06; % far down from the leg
% A = 0.05; % amplitude of curve
% T = 0.10; % width of curve

% SMALLER FOR ARDUINO
X = 0.16; % far out from leg
S = -0.06; % far down from the leg
A = 0.03; % amplitude of curve
T = 0.06; % width of curve

% pre-rotation offsets
x_offset = -0.02;
y_offset = 0.05;

% % HALF SWING HALF STANCE
% NUM_DATA_POINTS = 40;   
% half = NUM_DATA_POINTS / 2;
% 
% P1 = [-T/2, S];
% P2 = [0, S + 2*A];
% P3 = [T/2, S];
% 
% t = linspace(0, 1, half);
% xyz = zeros(NUM_DATA_POINTS, 3);  % each row: [x,y,z]
% 
% % swing
% for i = 1:half
%     xyz(i,1) = X;
%     xyz(i,2) = (1 - t(i))^2 * P1(1) + 2*(1 - t(i))*t(i)*P2(1) + t(i)^2 * P3(1);
%     xyz(i,3) = (1 - t(i))^2 * P1(2) + 2*(1 - t(i))*t(i)*P2(2) + t(i)^2 * P3(2);
% end
% 
% % stance
% y_stance = linspace(T/2, -T/2, half);
% for i = 1:half
%     xyz(half + i, 1) = X;
%     xyz(half + i, 2) = y_stance(i);
%     xyz(half + i, 3) = S;
% end

% QUARTER SWING
T_STALL = 2;
NUM_DATA_POINTS = 40;   
quarter = NUM_DATA_POINTS / 4;

P1 = [-T/2, S];
P2 = [0, S + 2*A];
P3 = [T/2, S];

t = linspace(0, 1, quarter);
xyz = zeros(NUM_DATA_POINTS + 2*T_STALL, 3); 

% swing
for i = 1:quarter
    xyz(i,1) = X;
    xyz(i,2) = (1 - t(i))^2 * P1(1) + 2*(1 - t(i))*t(i)*P2(1) + t(i)^2 * P3(1);
    xyz(i,3) = (1 - t(i))^2 * P1(2) + 2*(1 - t(i))*t(i)*P2(2) + t(i)^2 * P3(2);
end

for i = 1:T_STALL
    xyz(quarter + i, 1) = xyz(quarter + i - 1, 1);
    xyz(quarter + i, 2) = xyz(quarter + i - 1, 2);
    xyz(quarter + i, 3) = xyz(quarter + i - 1, 3);
end

% stance
y_stance = linspace(T/2, -T/2, 3*quarter);
for i = 1:3*quarter
    xyz(quarter + T_STALL + i, 1) = X;
    xyz(quarter + T_STALL + i, 2) = y_stance(i);
    xyz(quarter + T_STALL + i, 3) = S;
end

for i = 1:T_STALL
    xyz(4*quarter + T_STALL + i, 1) = xyz(4*quarter + T_STALL + i - 1, 1);
    xyz(4*quarter + T_STALL + i, 2) = xyz(4*quarter + T_STALL + i - 1, 2);
    xyz(4*quarter + T_STALL + i, 3) = xyz(4*quarter + T_STALL + i - 1, 3);
end

% rotation for front right leg
Rz = [cos(deg2rad(-45)) -sin(deg2rad(-45)) 0; sin(deg2rad(-45)) cos(deg2rad(-45)) 0; 0 0 1];

% xyzRz = zeros(NUM_DATA_POINTS, 3);
xyzRz = zeros(NUM_DATA_POINTS + 2*T_STALL, 3);
for i = 1:length(xyz)
    xyzRz(i, :) = Rz*[xyz(i, 1) + x_offset;xyz(i, 2) + y_offset;xyz(i, 3)];
end

xyz = xyzRz;

figure('Units','normalized','Position',[0.05 0.05 0.7 0.7]);
ax = axes('Parent',gcf);
% ax.Color = [0 0 0];
hold(ax,'on');
grid(ax,'on');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
view(135,25); axis equal; rotate3d on;

% draw bezier curve
plot3(ax, xyz(:,1), xyz(:,2), xyz(:,3), 'r.-', 'LineWidth', 1.2, 'MarkerSize', 8);

% leg base marker
scatter3(ax, 0, 0, 0, 80, 'y', 'filled');
text(0,0,0,'  Leg Base','FontWeight','bold','Color','b');

% start with home config display
axleg = show(leg, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
title(ax, '3DOF Leg - Following Bezier Trajectory');

% embellish link appearance (search for line objects and thicken)
linkLines = findobj(ax, 'Type', 'Line');
if ~isempty(linkLines)
    set(linkLines, 'LineWidth', 6, 'Color', 'y'); % yellow-ish
end

footMarker = plot3(ax, xyz(1,1), xyz(1,2), xyz(1,3), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);

theta1_min = deg2rad(-45); theta1_max = deg2rad(45);
theta2_min = deg2rad(-90); theta2_max = deg2rad(90);
theta4_min = deg2rad(-90); theta4_max = deg2rad(90);



% actual foot trajectory (as followed by FK)
actualTrace = plot3(ax, nan, nan, nan, 'm.-', 'LineWidth', 1.2, 'MarkerSize', 20); % cyan line

% === Setup video writer to save animation ===
% pauseTime = 0.01;  % pause between frames
%videoFilename = 'leg_bezier_trajectory.mp4';  % change to .avi if you prefer
%v = VideoWriter(videoFilename, 'MPEG-4');     % 'Motion JPEG AVI' also works
%v.FrameRate = 5;                  % match playback speed
%open(v);
disp('');
for i = 1:NUM_DATA_POINTS
    x = xyz(i,1);
    y = xyz(i,2);
    z = xyz(i,3);
    
    theta1 = atan2(y, x);   % hip joint

    LHS = ((x*cos(theta1) - L1 + y*sin(theta1))^2 + z^2-L2^2-L3^2-L4^2-2*L2*L3*cos(theta3))/(2*L4);
    A1 = L2*cos(theta3) + L3;
    B1 = L2*sin(theta3);
    phi1 = atan2(A1, B1);
    a1 = A1/sin(phi1);
    theta4 = phi1 - asin(LHS/a1);

    A2 = L2 + L4*cos(theta3+theta4) + L3*cos(theta3);
    B2 = L4*sin(theta3+theta4) + L3*sin(theta3);
    phi2 = atan2(B2, A2);
    a2 = B2/sin(phi2); 
    theta2 = asin(z/a2) + phi2;
    
    fprintf("Step %d\nThetas: %.2f, %.2f, %.2f\nCoordinates: %.2f, %.2f, %.2f\n", i-1, rad2deg(theta1), rad2deg(theta2), rad2deg(theta4), x, y, z);
    % check joint bounds and realness
    if isreal(theta1) && isreal(theta2) && isreal(theta4) && ...
            theta1 >= theta1_min && theta1 <= theta1_max && ... 
            theta2 >= theta2_min && theta2 <= theta2_max && ...
            theta4 >= theta4_min && theta4 <= theta4_max
        
        % apply to robot config
        config = homeConfiguration(leg); % making config to change thetas 
        config(1).JointPosition = theta1;
        config(2).JointPosition = theta2;
        config(3).JointPosition = theta3;
        config(4).JointPosition = theta4;
        
        % update the robot pose in the same axes
        show(leg, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);

        % Compute actual FK end-effector position (using MATLAB robotics toolbox)
        eeTform = getTransform(leg, config, 'foot'); 
        eePos = tform2trvec(eeTform);
        
        % Update the actual trace with new point
        actualTrace.XData(end+1) = eePos(1);
        actualTrace.YData(end+1) = eePos(2);
        actualTrace.ZData(end+1) = eePos(3);
        
        % update foot marker (magenta for reachable)
        set(footMarker, 'XData', x, 'YData', y, 'ZData', z, 'MarkerEdgeColor','m','MarkerFaceColor','m');
    else
        % unreachable due to joint bounds
        disp("Invalid point")
        set(footMarker, 'XData', x, 'YData', y, 'ZData', z, 'MarkerEdgeColor','k','MarkerFaceColor','k');
    end
    
    linkLines = findobj(axleg, 'Type', 'Line');
    set(linkLines, 'LineWidth', 10, 'Color', 'y');  % yellow, thick links 
    drawnow;
    
    %frame = getframe(gcf);
    %writeVideo(v, frame);

    %pause(pauseTime);
end
%close(v);
%disp(['Animation saved to ', videoFilename]);
disp('Animation done.');

%% MODELING LEFT LEG (+ TESTING KINEMATICS):

%{ 
asin solution reaches target but leg is not the right shape. x = 0.16; y = 
-0.07; z = -0.05; these values give a point which can be reached in the joint 
bounds, but the bezier needs to be shifted with some constants so that it follows 
closely.

both acos being +ve reaches the point above but leg is same incorrect shape 
as before. bezier is also same as before. i checked and they give the exact 
same workspace.

acos of theta4 being -ve and theta2 being +ve reaches the target with correct 
leg shape. the bezier is also fully tracing with the same constants used for 
the right leg.

both acos being -ve doesnt give a sensible workspace so thats out of the consideration

when theta4 +ve and theta2 -ve, workspace:



when theta4 -ve and theta2 +ve, workspace:



asin and both +ve acos:



since the  theta4 -ve and theta2 +ve set makes the leg reach the target in 
correct orientation and simulates the bezier curve correctly, we can move forward 
with this for left leg.
%}

clc; clear; close all;

t1 = deg2rad(0);  
t2 = deg2rad(54); 
tfixed = deg2rad(45);  
t4 = deg2rad(90);

L1 = 0.02845;    % hip length
L2 = 0.05439;   % knee length0
L3 = 0.02637;   % knee length1
L4 = 0.09265;    % ankle length

theta1_min = -45;  theta1_max = 45;
theta2_min = -90;  theta2_max = 90;
theta4_min = -90;  theta4_max = 90;

dhparams = [L1    -pi/2   0    t1;      % Link 1 (hip)
            L2     pi     0    t2;      % Link 20 (knee)
            L3     0      0    tfixed;  % Link 21 (bend)
            L4     0      0    t4];     % Link 3 (ankle)

leg = rigidBodyTree; % making leg structure

% assigning frames, base is hip_motor
knee = rigidBody('knee_motor');
bend = rigidBody('bend');
ankle = rigidBody('ankle_motor');
foot = rigidBody('foot');

kneejnt = rigidBodyJoint('kneejnt','revolute');
bendjnt = rigidBodyJoint('bendjnt', 'revolute');
anklejnt = rigidBodyJoint('anklejnt','revolute');
footjnt = rigidBodyJoint('footjnt','revolute');

setFixedTransform(kneejnt,dhparams(1,:),'dh'); % using DH to set the frame on the body
setFixedTransform(bendjnt,dhparams(2,:),'dh');
setFixedTransform(anklejnt,dhparams(3,:),'dh');
setFixedTransform(footjnt,dhparams(4,:),'dh');

knee.Joint = kneejnt;
bend.Joint = bendjnt;
ankle.Joint = anklejnt;
foot.Joint = footjnt;

addBody(leg,knee,'base')
addBody(leg,bend,'knee_motor')
addBody(leg,ankle,'bend')
addBody(leg,foot,'ankle_motor')

% testing kinematics
x = 0.16;
y = -0.07;
z = -0.05;

theta3 = -pi/4;
theta1 = atan2(y, x);   % hip joint angle
d1 = rad2deg(theta1)

if isreal(theta1) && d1 >= theta1_min && d1 <= theta1_max
    
    LHS = ((x*cos(theta1) - L1 + y*sin(theta1))^2 + z^2-L2^2-L3^2-L4^2-2*L2*L3*cos(theta3))/(2*L4);
    A1 = L2*cos(theta3) + L3;
    B1 = L2*sin(theta3);
    phi1 = atan2(B1, A1);    
    a1 = B1/sin(phi1);
    theta4 = -acos(LHS/a1) - phi1;
    d4 = rad2deg(theta4)

    if isreal(theta4) && d4 >= theta4_min && d4 <= theta4_max

        A2 = L2 + L4*cos(theta3+theta4) + L3*cos(theta3);
        B2 = L4*sin(theta3+theta4) + L3*sin(theta3);
        phi2 = atan2(A2, B2);    % cos sol
        a2 = A2/sin(phi2);
        theta2 = acos(z/a2) - phi2;
        d2 = rad2deg(theta2)

        if isreal(theta2) && d2 >= theta2_min && d2 <= theta2_max

            config = homeConfiguration(leg); % making config to change thetas    
            config(1).JointPosition = theta1;   % hip joint angle
            config(2).JointPosition = theta2;   % knee joint angle
            config(3).JointPosition = theta3;   % ankle joint angle
            config(4).JointPosition = theta4;   % ankle joint angle            
            
            figure('Units','normalized','Position',[0.05 0.05 0.7 0.7]);
            % figure('Color','k','Units','normalized','Position',[0.05 0.05 0.7 0.7]);
            ax = axes('Parent', gcf);
            % ax.Color = [0 0 0];
            hold(ax, 'on');
            grid(ax, 'on');
            xlabel(ax, 'x (m)'); 
            ylabel(ax, 'y (m)'); 
            zlabel(ax, 'z (m)');            
            view(ax, 135, 25);
            axis(ax, 'equal');
            rotate3d on;
            
            % draw leg
            show(leg, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
            title(ax, '3DOF Leg in Configured Pose');   
            text(0,0,0,'  Leg Base','FontWeight','bold','Color','b');
            drawnow;
            
            linkLines = findobj(ax, 'Type', 'Line');
            set(linkLines, 'LineWidth', 15, 'Color', 'y');  % yellow, thick links
            
            % draw target
            hold on
            plot3(x, y, z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % red dot for target
            text(x, y, z, '  Target', 'Color', 'r', 'FontWeight', 'bold');
        end
    end
end
%% Bezier curve generation and rotation for back left leg

X = 0.16; % far out from leg
S = -0.06; % far down from the leg
A = 0.03; % amplitude of curve
T = 0.06; % width of curve
x_offset = -0.02;
y_offset = -0.05; % note that this sign changed

NUM_DATA_POINTS = 40;   
half = NUM_DATA_POINTS / 2;

P1 = [-T/2, S];
P2 = [0, S + 2*A];
P3 = [T/2, S];

t = linspace(0, 1, half);
xyz = zeros(NUM_DATA_POINTS, 3);  % each row: [x,y,z]

% Swing phase (Bezier curve)
for i = 1:half
    xyz(i,1) = X;
    xyz(i,2) = (1 - t(i))^2 * P1(1) + 2*(1 - t(i))*t(i)*P2(1) + t(i)^2 * P3(1);
    xyz(i,3) = (1 - t(i))^2 * P1(2) + 2*(1 - t(i))*t(i)*P2(2) + t(i)^2 * P3(2);
end

y_stance = linspace(T/2, -T/2, half);
for i = 1:half
    xyz(half + i, 1) = X;
    xyz(half + i, 2) = y_stance(i);
    xyz(half + i, 3) = S;
end

Rz = [cos(deg2rad(45)) -sin(deg2rad(45)) 0; sin(deg2rad(45)) cos(deg2rad(45)) 0; 0 0 1];

xyzRz = zeros(NUM_DATA_POINTS, 3);
for i = 1:length(xyz)
    xyzRz(i, :) = Rz*[xyz(i, 1) + x_offset;xyz(i, 2) + y_offset;xyz(i, 3)];
end

xyz = xyzRz;

figure('Units','normalized','Position',[0.05 0.05 0.7 0.7]);
% figure('Color','k','Units','normalized','Position',[0.05 0.05 0.7 0.7]);
ax = axes('Parent',gcf);
% ax.Color = [0 0 0];
hold(ax,'on');
grid(ax,'on');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
view(135,25); axis equal; rotate3d on;

% draw curve
plot3(ax, xyz(:,1), xyz(:,2), xyz(:,3), 'r.-', 'LineWidth', 1.2, 'MarkerSize', 8);

% leg base marker
scatter3(ax, 0, 0, 0, 80, 'y', 'filled');
text(0,0,0,'  Leg Base','FontWeight','bold','Color','b');

% Start with home config display
axleg = show(leg, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
title(ax, '3DOF Leg - Following Bezier Trajectory');

% embellish link appearance (search for line objects and thicken)
linkLines = findobj(ax, 'Type', 'Line');
if ~isempty(linkLines)
    set(linkLines, 'LineWidth', 6, 'Color', 'y'); % yellow-ish
end

footMarker = plot3(ax, xyz(1,1), xyz(1,2), xyz(1,3), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);

theta1_min = deg2rad(-45); theta1_max = deg2rad(45);
theta2_min = deg2rad(-90); theta2_max = deg2rad(90);
theta4_min = deg2rad(-90); theta4_max = deg2rad(90);

% actual foot trajectory (as followed by FK)
actualTrace = plot3(ax, nan, nan, nan, 'm.-', 'LineWidth', 1.2, 'MarkerSize', 20); % cyan line

% === Setup video writer ===
% pauseTime = 0.1;  % pause between frames
%videoFilename = 'leg_bezier_trajectory.mp4';  % change to .avi if you prefer
%v = VideoWriter(videoFilename, 'MPEG-4');     % 'Motion JPEG AVI' also works
%v.FrameRate = 5;                  % match playback speed
%open(v);

for i = 1:NUM_DATA_POINTS
    x = xyz(i,1);
    y = xyz(i,2);
    z = xyz(i,3);
    
    theta1 = atan2(y, x);   % hip joint
    % dtheta1 = rad2deg(theta1)
    
    LHS = ((x*cos(theta1) - L1 + y*sin(theta1))^2 + z^2-L2^2-L3^2-L4^2-2*L2*L3*cos(theta3))/(2*L4);
    A1 = L2*cos(theta3) + L3;
    B1 = L2*sin(theta3);
    phi1 = atan2(B1, A1);   
    a1 = B1/sin(phi1);
    theta4 = -acos(LHS/a1) - phi1;

    A2 = L2 + L4*cos(theta3+theta4) + L3*cos(theta3);
    B2 = L4*sin(theta3+theta4) + L3*sin(theta3);
    phi2 = atan2(A2, B2);  
    a2 = A2/sin(phi2);
    theta2 = acos(z/a2) - phi2;
    
    % check joint bounds and realness
    if isreal(theta1) && isreal(theta2) && isreal(theta4) && ...
            theta1 >= theta1_min && theta1 <= theta1_max && ...
            theta2 >= theta2_min && theta2 <= theta2_max && ...
            theta4 >= theta4_min && theta4 <= theta4_max
        
        % apply to robot config
        config = homeConfiguration(leg); % making config to change thetas 
        config(1).JointPosition = theta1;
        config(2).JointPosition = theta2;
        config(3).JointPosition = theta3;
        config(4).JointPosition = theta4;
        
        % update the robot pose in the same axes
        show(leg, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);

        % Compute actual FK end-effector position (using MATLAB robotics toolbox)
        eeTform = getTransform(leg, config, 'foot'); 
        eePos = tform2trvec(eeTform);
        
        % Update the actual trace with new point
        actualTrace.XData(end+1) = eePos(1);
        actualTrace.YData(end+1) = eePos(2);
        actualTrace.ZData(end+1) = eePos(3);
        
        % update foot marker (magenta for reachable)
        set(footMarker, 'XData', x, 'YData', y, 'ZData', z, 'MarkerEdgeColor','m','MarkerFaceColor','m');
    else
        % unreachable due to joint bounds
        set(footMarker, 'XData', x, 'YData', y, 'ZData', z, 'MarkerEdgeColor','k','MarkerFaceColor','k');
    end
    
    linkLines = findobj(axleg, 'Type', 'Line');
    set(linkLines, 'LineWidth', 10, 'Color', 'y');  % yellow, thick links 
    drawnow;
    
    %frame = getframe(gcf);
    %writeVideo(v, frame);

    %pause(pauseTime);
end
%close(v);
%disp(['Animation saved to ', videoFilename]);
%disp('Animation done.');

%% WHOLE ROBOT MODEL:

clc; clear; close all;

LDF = 0.0987;   % euclidean distance of front legs hip from robot base
LDB = 0.1377;   % euclidean distance of back legs hip from robot base
L1 = 0.02845;    % hip length
L2 = 0.05439;   % knee length0
L3 = 0.02637;   % knee length1
L4 = 0.09265;    % ankle length

body = rigidBodyTree; % making body structure

% ASSUMING FRONT RIGHT LEG MOTIONS
rft1 = deg2rad(0);  rft2 = deg2rad(0);  rft3 = deg2rad(0);  rft4 = deg2rad(0);  rft = deg2rad(0);

rfdhparams = [LDF     0      0    rft
              L1     pi/2   0    rft1;      % Link 1 (hip)
              L2     pi     0    rft2;      % Link 20 (knee)
              L3     0      0    rft3;  % Link 21 (bend)
              L4     0      0    rft4];     % Link 3 (ankle)

rfbase = rigidBody('rf_base');
rfleg = rigidBody('rfhip_motor');
rfhip = rigidBody('rfknee_motor');
rfknee = rigidBody('rfankle_motor');
rfankle = rigidBody('rffoot');

rfbasejnt = rigidBodyJoint('rfbase', 'revolute')
rflegjnt = rigidBodyJoint('rflegjnt', 'revolute')
rfhipjnt = rigidBodyJoint('rfhipjnt','revolute');
rfkneejnt = rigidBodyJoint('rfkneejnt','revolute');
rfanklejnt = rigidBodyJoint('rfanklejnt','revolute');

setFixedTransform(rfbasejnt,rfdhparams(1,:),'dh' )
setFixedTransform(rflegjnt,rfdhparams(2,:),'dh' )
setFixedTransform(rfhipjnt,rfdhparams(3,:),'dh'); 
setFixedTransform(rfkneejnt,rfdhparams(4,:),'dh');
setFixedTransform(rfanklejnt,rfdhparams(5,:),'dh');

rfbase.Joint = rfbasejnt;
rfleg.Joint = rflegjnt;
rfhip.Joint = rfhipjnt;
rfknee.Joint = rfkneejnt;
rfankle.Joint = rfanklejnt;

addBody(body,rfbase, 'base')
addBody(body,rfleg, 'rf_base')
addBody(body,rfhip,'rfhip_motor')
addBody(body,rfknee,'rfknee_motor')
addBody(body,rfankle,'rfankle_motor')

% ASSUMING BACK RIGHT LEG MOTIONS

rbt1 = deg2rad(0);  rbt2 = deg2rad(0);  rbt3 = deg2rad(0);  rbt4 = deg2rad(0);  rbt = deg2rad(0);

rbdhparams = [LDB     0      0    rbt
              L1     pi/2   0    rbt1;      % Link 1 (hip)
              L2     pi     0    rbt2;      % Link 20 (knee)
              L3     0      0    rbt3;  % Link 21 (bend)
              L4     0      0    rbt4];     % Link 3 (ankle)

rbbase = rigidBody('rb_base');
rbleg = rigidBody('rbhip_motor');
rbhip = rigidBody('rbknee_motor');
rbknee = rigidBody('rbankle_motor');
rbankle = rigidBody('rbfoot');

rbbasejnt = rigidBodyJoint('rbbase', 'revolute')
rblegjnt = rigidBodyJoint('rblegjnt', 'revolute')
rbhipjnt = rigidBodyJoint('rbhipjnt','revolute');
rbkneejnt = rigidBodyJoint('rbkneejnt','revolute');
rbanklejnt = rigidBodyJoint('rbanklejnt','revolute');

setFixedTransform(rbbasejnt,rbdhparams(1,:),'dh' )
setFixedTransform(rblegjnt,rbdhparams(2,:),'dh' )
setFixedTransform(rbhipjnt,rbdhparams(3,:),'dh'); 
setFixedTransform(rbkneejnt,rbdhparams(4,:),'dh');
setFixedTransform(rbanklejnt,rbdhparams(5,:),'dh');

rbbase.Joint = rbbasejnt;
rbleg.Joint = rblegjnt;
rbhip.Joint = rbhipjnt;
rbknee.Joint = rbkneejnt;
rbankle.Joint = rbanklejnt;

addBody(body,rbbase, 'base')
addBody(body,rbleg, 'rb_base')
addBody(body,rbhip,'rbhip_motor')
addBody(body,rbknee,'rbknee_motor')
addBody(body,rbankle,'rbankle_motor')

% LEFT FRONT OF ROBOT

lft1 = deg2rad(0);  lft2 = deg2rad(0);  lft3 = deg2rad(0);  lft4 = deg2rad(0);  lft = deg2rad(0);

lfdhparams = [LDF     0      0    lft
              L1    -pi/2   0    lft1;      % Link 1 (hip)
              L2     pi     0    lft2;      % Link 20 (knee)
              L3     0      0    lft3;  % Link 21 (bend)
              L4     0      0    lft4];     % Link 3 (ankle)

lfbase = rigidBody('lf_base');
lfleg = rigidBody('lfhip_motor');
lfhip = rigidBody('lfknee_motor');
lfknee = rigidBody('lfankle_motor');
lfankle = rigidBody('lffoot');

lfbasejnt = rigidBodyJoint('lfbase', 'revolute')
lflegjnt = rigidBodyJoint('lflegjnt', 'revolute')
lfhipjnt = rigidBodyJoint('lfhipjnt','revolute');
lfkneejnt = rigidBodyJoint('lfkneejnt','revolute');
lfanklejnt = rigidBodyJoint('lfanklejnt','revolute');

setFixedTransform(lfbasejnt,lfdhparams(1,:),'dh' )
setFixedTransform(lflegjnt,lfdhparams(2,:),'dh' )
setFixedTransform(lfhipjnt,lfdhparams(3,:),'dh'); 
setFixedTransform(lfkneejnt,lfdhparams(4,:),'dh');
setFixedTransform(lfanklejnt,lfdhparams(5,:),'dh');

lfbase.Joint = lfbasejnt;
lfleg.Joint = lflegjnt;
lfhip.Joint = lfhipjnt;
lfknee.Joint = lfkneejnt;
lfankle.Joint = lfanklejnt;

addBody(body,lfbase, 'base')
addBody(body,lfleg, 'lf_base')
addBody(body,lfhip,'lfhip_motor')
addBody(body,lfknee,'lfknee_motor')
addBody(body,lfankle,'lfankle_motor')

% LEFT BACK OF ROBOT

lbt1 = deg2rad(0);  lbt2 = deg2rad(0);  lbt3 = deg2rad(0);  lbt4 = deg2rad(0);  lbt = deg2rad(0);

lbdhparams = [LDB     0      0    lbt
              L1    -pi/2   0    lbt1;      % Link 1 (hip)
              L2     pi     0    lbt2;      % Link 20 (knee)
              L3     0      0    lbt3;  % Link 21 (bend)
              L4     0      0    lbt4];     % Link 3 (ankle)

lbbase = rigidBody('lb_base');
lbleg = rigidBody('lbhip_motor');
lbhip = rigidBody('lbknee_motor');
lbknee = rigidBody('lbankle_motor');
lbankle = rigidBody('lbfoot');

lbbasejnt = rigidBodyJoint('lbbase', 'revolute')
lblegjnt = rigidBodyJoint('lblegjnt', 'revolute')
lbhipjnt = rigidBodyJoint('lbhipjnt','revolute');
lbkneejnt = rigidBodyJoint('lbkneejnt','revolute');
lbanklejnt = rigidBodyJoint('lbanklejnt','revolute');

setFixedTransform(lbbasejnt,lbdhparams(1,:),'dh' )
setFixedTransform(lblegjnt,lbdhparams(2,:),'dh' )
setFixedTransform(lbhipjnt,lbdhparams(3,:),'dh'); 
setFixedTransform(lbkneejnt,lbdhparams(4,:),'dh');
setFixedTransform(lbanklejnt,lbdhparams(5,:),'dh');

lbbase.Joint = lbbasejnt;
lbleg.Joint = lblegjnt;
lbhip.Joint = lbhipjnt;
lbknee.Joint = lbkneejnt;
lbankle.Joint = lbanklejnt;

addBody(body,lbbase, 'base')
addBody(body,lbleg, 'lb_base')
addBody(body,lbhip,'lbhip_motor')
addBody(body,lbknee,'lbknee_motor')
addBody(body,lbankle,'lbankle_motor')

showdetails(body)  % shows rigidbodytree with properties
config = homeConfiguration(body); % making config to change thetas

% config(1).JointPosition = rft1;   % hip joint angle
% config(2).JointPosition = rft2;   % knee joint angle
% config(3).JointPosition = rft3;   % ankle joint angle
% config(4).JointPosition = rbt1;   % hip joint angle
% config(5).JointPosition = rbt2;   % knee joint angle
% config(6).JointPosition = rbt3;   % ankle joint angle
% config(7).JointPosition = lft1;   % hip joint angle
% config(8).JointPosition = lft2;   % knee joint angle
% config(9).JointPosition = lft3;   % ankle joint angle
% config(10).JointPosition = lbt1;   % hip joint angle
% config(11).JointPosition = lbt2;   % knee joint angle
% config(12).JointPosition = lbt3;   % ankle joint angle

% config(1).JointPosition = deg2rad(45);  
% config(2).JointPosition = deg2rad(0);  
% config(3).JointPosition = deg2rad(0);  
% config(4).JointPosition = deg2rad(45);  
% config(5).JointPosition = deg2rad(0);  
% config(6).JointPosition = deg2rad(-45);   
% config(7).JointPosition = deg2rad(0);   
% config(8).JointPosition = deg2rad(0);  
% config(9).JointPosition = deg2rad(45);  
% config(10).JointPosition = deg2rad(0); 
% config(11).JointPosition = deg2rad(135);   
% config(12).JointPosition = deg2rad(0);  
% config(13).JointPosition = deg2rad(0); 
% config(14).JointPosition = deg2rad(-45);  
% config(15).JointPosition = deg2rad(0);  
% config(16).JointPosition = deg2rad(-135);  
% config(17).JointPosition = deg2rad(0); 
% config(18).JointPosition = deg2rad(0);  
% config(19).JointPosition = deg2rad(-45);   
% config(20).JointPosition = deg2rad(0);

config(1).JointPosition = deg2rad(45);  
config(2).JointPosition = -0.673;  
config(3).JointPosition = 0.1981;  
config(4).JointPosition = deg2rad(45);  
config(5).JointPosition = -0.4008;  
config(6).JointPosition = deg2rad(-45);   
config(7).JointPosition = -0.7032;   
config(8).JointPosition = 0.0056;  
config(9).JointPosition = deg2rad(45);  
config(10).JointPosition = 1.475; 
config(11).JointPosition = deg2rad(135);   
config(12).JointPosition = 0.7781;  
config(13).JointPosition = -0.3264; 
config(14).JointPosition = deg2rad(-45);  
config(15).JointPosition = 0.3947;  
config(16).JointPosition = deg2rad(-135);  
config(17).JointPosition = deg2rad(0); 
config(18).JointPosition = deg2rad(0);  
config(19).JointPosition = deg2rad(-45);   
config(20).JointPosition = deg2rad(0);

figure('Units','normalized','Position',[0.05 0.05 0.7 0.7]);
% figure('Color','k','Units','normalized','Position',[0.05 0.05 0.7 0.7]);
ax = axes('Parent', gcf);
% ax.Color = [0 0 0];
hold(ax, 'on');
grid(ax, 'on');
xlabel(ax, 'x (m)'); 
ylabel(ax, 'y (m)'); 
zlabel(ax, 'z (m)');            
view(ax, 135, 25);
axis(ax, 'equal');
rotate3d on;

show(body, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
title(ax, '3DOF Robot in Configured Pose');           
drawnow;

linkLines = findobj(ax, 'Type', 'Line');
set(linkLines, 'LineWidth', 5, 'Color', 'y');  % yellow, thick links

%% BEZIER TRAJECTORY OF WHOLE ROBOT:

X = 0.16; % far out from leg
S = -0.06; % far down from the leg
A = 0.05; % amplitude of curve
T = 0.10; % width of curve
x_offset = -0.02;
y_offset = 0.05;

NUM_DATA_POINTS = 20;   
half = NUM_DATA_POINTS / 2;

P1 = [-T/2, S];
P2 = [0, S + 2*A];
P3 = [T/2, S];

t = linspace(0, 1, half);
xyz = zeros(NUM_DATA_POINTS, 3); 

theta1_min = deg2rad(-45); theta1_max = deg2rad(45);
theta2_min = deg2rad(-90); theta2_max = deg2rad(90);
theta4_min = deg2rad(-90); theta4_max = deg2rad(90);

% swing
for i = 1:half
    xyz(i,1) = X;
    xyz(i,2) = (1 - t(i))^2 * P1(1) + 2*(1 - t(i))*t(i)*P2(1) + t(i)^2 * P3(1);
    xyz(i,3) = (1 - t(i))^2 * P1(2) + 2*(1 - t(i))*t(i)*P2(2) + t(i)^2 * P3(2);
end

% stance
y_stance = linspace(T/2, -T/2, half);
for i = 1:half
    xyz(half + i, 1) = X;
    xyz(half + i, 2) = y_stance(i);
    xyz(half + i, 3) = S;
end

% rotation for rf leg
Rf = [cos(deg2rad(-45)) -sin(deg2rad(-45)) 0; sin(deg2rad(-45)) cos(deg2rad(-45)) 0; 0 0 1];
xyzRf = zeros(NUM_DATA_POINTS, 3);
for i = 1:length(xyz)
    xyzRf(i, :) = Rf*[xyz(i, 1) + x_offset;xyz(i, 2) + y_offset;xyz(i, 3)];
end

% rotation for rb leg
Rb = [cos(deg2rad(45)) -sin(deg2rad(45)) 0; sin(deg2rad(45)) cos(deg2rad(45)) 0; 0 0 1];
xyzRb = zeros(NUM_DATA_POINTS, 3);
for i = 1:length(xyz)
    xyzRb(i, :) = Rb*[xyz(i, 1) + x_offset;xyz(i, 2) - y_offset;xyz(i, 3)];
end

% rotation for lb leg
Lb = [cos(deg2rad(-45)) -sin(deg2rad(-45)) 0; sin(deg2rad(-45)) cos(deg2rad(-45)) 0; 0 0 1];
xyzLb = zeros(NUM_DATA_POINTS, 3);
for i = 1:length(xyz)
    xyzLb(i, :) = Lb*[xyz(i, 1) + x_offset;xyz(i, 2) + y_offset;xyz(i, 3)];
end

% rotation for lf leg
Lf = [cos(deg2rad(45)) -sin(deg2rad(45)) 0; sin(deg2rad(45)) cos(deg2rad(45)) 0; 0 0 1];
xyzLf = zeros(NUM_DATA_POINTS, 3);
for i = 1:length(xyz)
    xyzLf(i, :) = Lf*[xyz(i, 1) + x_offset;xyz(i, 2) - y_offset;xyz(i, 3)];
end

%% Visualize entire robot and all legs following trajectory

figure('Units','normalized','Position',[0.05 0.05 0.7 0.7]);
% figure('Color','k','Units','normalized','Position',[0.05 0.05 0.7 0.7]);
ax = axes('Parent',gcf);
% ax.Color = [0 0 0];
hold(ax,'on');
grid(ax,'on');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
view(135,25); axis equal; rotate3d on;
xlim([-0.3 0.3]);
ylim([-0.3 0.3]);
zlim([-0.2 0.2]);

% % draw workspace trajectory points
% plot3(ax, xyz(:,1), xyz(:,2), xyz(:,3), 'r.-', 'LineWidth', 1.2, 'MarkerSize', 8);
% leg base marker
scatter3(ax, 0, 0, 0, 80, 'y', 'filled');
text(0,0,0,'Robot Base','FontWeight','bold','Color','b');

% Start with home config display
axbody = show(body, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
title(ax, '3DOF Robot Following Bezier Trajectory');

% footMarker = plot3(ax, xyz(1,1), xyz(1,2), xyz(1,3), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
% 
% actual foot trajectory (as followed by FK)
% actualTrace = plot3(ax, nan, nan, nan, 'm.-', 'LineWidth', 1.2, 'MarkerSize', 20); % cyan line

% config = homeConfiguration(body); % making config to change thetas

for i = 1:NUM_DATA_POINTS
    % constants
    theta3R = pi/4;  % for right side
    theta3L = -pi/4;  % for left side
    
    % FRONT RIGHT LEG
    x = xyzRf(i,1);
    y = xyzRf(i,2);
    z = xyzRf(i,3);

    theta1Rf = atan2(y, x);  % hip joint 
    
    LHS = ((x*cos(theta1Rf) - L1 + y*sin(theta1Rf))^2 + z^2-L2^2-L3^2-L4^2-2*L2*L3*cos(theta3R))/(2*L4);
    A1 = L2*cos(theta3R) + L3;
    B1 = L2*sin(theta3R);

    phi1 = atan2(A1, B1);    % sin sol
    a1 = A1/sin(phi1);
    theta4Rf = phi1 - asin(LHS/a1);

    A2 = L2 + L4*cos(theta3R+theta4Rf) + L3*cos(theta3R);
    B2 = L4*sin(theta3R+theta4Rf) + L3*sin(theta3R);
    phi2 = atan2(B2, A2);    % sin sol
    a2 = B2/sin(phi2); 
    theta2Rf = phi2 + asin(z/a2);
    
    % BACK RIGHT LEG
    x = xyzRb(i,1);
    y = xyzRb(i,2);
    z = xyzRb(i,3);

    theta1 = rad2deg(theta1Rf)
    theta2 = rad2deg(theta2Rf)
    theta4 = rad2deg(theta4Rf)

    theta1Rb = atan2(y, x);   % hip joint 
    
    LHS = ((x*cos(theta1Rb) - L1 + y*sin(theta1Rb))^2 + z^2-L2^2-L3^2-L4^2-2*L2*L3*cos(theta3R))/(2*L4);
    A1 = L2*cos(theta3R) + L3;
    B1 = L2*sin(theta3R);

    phi1 = atan2(A1, B1);    % sin sol
    a1 = A1/sin(phi1);
    theta4Rb = phi1 - asin(LHS/a1);

    A2 = L2 + L4*cos(theta3R+theta4Rb) + L3*cos(theta3R);
    B2 = L4*sin(theta3R+theta4Rb) + L3*sin(theta3R);
    phi2 = atan2(B2, A2);    % sin sol
    a2 = B2/sin(phi2); 
    theta2Rb = phi2 + asin(z/a2);
    
    % FRONT LEFT LEG
    x = xyzLf(i,1);
    y = xyzLf(i,2);
    z = xyzLf(i,3);

    theta1Lf = atan2(y, x);   % hip joint 

    LHS = ((x*cos(theta1Lf) - L1 + y*sin(theta1Lf))^2 + z^2-L2^2-L3^2-L4^2-2*L2*L3*cos(theta3L))/(2*L4);
    A1 = L2*cos(theta3L) + L3;
    B1 = L2*sin(theta3L);

    phi1 = atan2(B1, A1);    % cos sol
    a1 = B1/sin(phi1);
    theta4Lf = -acos(LHS/a1) - phi1;

    A2 = L2 + L4*cos(theta3L+theta4Lf) + L3*cos(theta3L);
    B2 = L4*sin(theta3L+theta4Lf) + L3*sin(theta3L);

    phi2 = atan2(A2, B2);    % cos sol
    a2 = A2/sin(phi2);
    theta2Lf = acos(z/a2) - phi2;    
    % % dtheta2 = rad2deg(theta2Lf)
    
    % BACK LEFT LEG
    x = xyzLb(i,1);
    y = xyzLb(i,2);
    z = xyzLb(i,3);

    theta1Lb = atan2(y, x);   % hip joint 

    LHS = ((x*cos(theta1Lb) - L1 + y*sin(theta1Lb))^2 + z^2-L2^2-L3^2-L4^2-2*L2*L3*cos(theta3L))/(2*L4);
    A1 = L2*cos(theta3L) + L3;
    B1 = L2*sin(theta3L);

    phi1 = atan2(B1, A1);    % cos sol
    a1 = B1/sin(phi1);
    theta4Lb = -acos(LHS/a1) - phi1;

    A2 = L2 + L4*cos(theta3L+theta4Lb) + L3*cos(theta3L);
    B2 = L4*sin(theta3L+theta4Lb) + L3*sin(theta3L);

    phi2 = atan2(A2, B2);    % cos sol
    a2 = A2/sin(phi2);
    theta2Lb = acos(z/a2) - phi2;    
    % % dtheta2 = rad2deg(theta2Lf)

    % check joint bounds and realness
    if isreal(theta1Rf) && isreal(theta2Rf) && isreal(theta4Rf) && ...
            theta1Rf >= theta1_min && theta1Rf <= theta1_max && ...
            theta2Rf >= theta2_min && theta2Rf <= theta2_max && ...
            theta4Rf >= theta4_min && theta4Rf <= theta4_max && ...
            isreal(theta1Rb) && isreal(theta2Rb) && isreal(theta4Rb) && ...
            theta1Rb >= theta1_min && theta1Rb <= theta1_max && ...
            theta2Rb >= theta2_min && theta2Rb <= theta2_max && ...
            theta4Rb >= theta4_min && theta4Rb <= theta4_max && ...
            isreal(theta1Lf) && isreal(theta2Lf) && isreal(theta4Lf) && ...
            theta1Lf >= theta1_min && theta1Lf <= theta1_max && ...
            theta2Lf >= theta2_min && theta2Lf <= theta2_max && ...
            theta4Lf >= theta4_min && theta4Lf <= theta4_max && ...
            isreal(theta1Lb) && isreal(theta2Lb) && isreal(theta4Lb) && ...
            theta1Lb >= theta1_min && theta1Lb <= theta1_max && ...
            theta2Lb >= theta2_min && theta2Lb <= theta2_max && ...
            theta4Lb >= theta4_min && theta4Lb <= theta4_max
        
        
        config = homeConfiguration(body); % making config to change thetas
        
        % apply trajectory angles to robot config
        config(1).JointPosition = deg2rad(45);
        config(2).JointPosition = theta1Rf;
        config(3).JointPosition = theta2Rf;
        config(4).JointPosition = theta3R;
        config(5).JointPosition = theta4Rf;
        config(6).JointPosition = deg2rad(-45);   
        config(7).JointPosition = theta1Rb;   
        config(8).JointPosition = theta2Rb;  
        config(9).JointPosition = theta3R;  
        config(10).JointPosition = theta4Rb;
        config(11).JointPosition = deg2rad(135);   
        config(12).JointPosition = theta1Lf;  
        config(13).JointPosition = theta2Lf; 
        config(14).JointPosition = theta3L;  
        config(15).JointPosition = theta4Lf;  
        config(16).JointPosition = deg2rad(-135);  
        config(17).JointPosition = theta1Lb; 
        config(18).JointPosition = theta2Lb;  
        config(19).JointPosition = theta3L;   
        config(20).JointPosition = theta4Lb;
        
        % update the robot pose in the same axes
        show(body, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
    end
        
    linkLines = findobj(axbody, 'Type', 'Line');
    set(linkLines, 'LineWidth', 10, 'Color', 'y');  % yellow, thick links 
    drawnow;
end


%% LEGACY
% ik = inverseKinematics('RigidBodyTree', leg);
% 
% % Choose weights for position/orientation tracking
% % [x y z roll pitch yaw]
% weights = [1 1 1 0 0 0]; % prioritize position only
% 
% % Pick target body and initial guess
% endEffector = 'foot';
% initialGuess = leg.homeConfiguration;
% 
% %% --- Define desired target position (in base frame) ---
% T_target = trvec2tform([0.12 0.02 -0.05]);  % target (x,y,z) in meters
% 
% %% Solve IK
% [configSol, solInfo] = ik(endEffector, T_target, weights, initialGuess);
% 
% %% Display results
% disp('Inverse Kinematics Solution (radians):');
% disp([configSol.JointPosition]);
% disp('Solution info:');
% disp(solInfo.Status);
% 
% %%
% %% --- Visualize result ---
% figure;
% show(leg, configSol, 'Frames', 'on', 'PreservePlot', false);
% title('3-DOF Leg After Inverse Kinematics');
% view(135, 25);
% axis equal;
% 
% T_init = getTransform(leg, config, 'foot');
% for s = linspace(0,1,30)
%     T = trinterp(T_init, T_target, s);
%     [cfg, ~] = ik(endEffector, T, weights, config);
%     show(leg, cfg, 'Frames', 'on', 'PreservePlot', false);
%     drawnow;
% end
% %% 
% % Peter Corke toolbox, but im not using this
% 
% clc; close all; clear;
% 
% L1 = 0.031;   % hip length
% L2 = 0.071;   % knee length 
% L3 = 0.088;   % ankle length
% 
% % [theta d a1 alpha]   give 0 for theta to initialize
% L(1) = Link([0 0 L1 pi/2]);
% L(2) = Link([0 0 L2 0]);
% L(3) = Link([0 0 L3 0]);
% 
% Robot = SerialLink(L);
% Robot.name = 'hexapod leg'
% Robot.plot([0 0 0])
% 
% q = [0 0 0];                % joint angles [hip knee ankle]
% T = Robot.fkine(q)          % homogeneous transform base→foot

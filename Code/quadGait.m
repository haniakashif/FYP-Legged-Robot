%% CRAWL GAIT (1)

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

config(1).JointPosition = deg2rad(45);  
config(2).JointPosition = deg2rad(0);  
config(3).JointPosition = deg2rad(0);  
config(4).JointPosition = deg2rad(45);  
config(5).JointPosition = deg2rad(0);  
config(6).JointPosition = deg2rad(-45);   
config(7).JointPosition = deg2rad(0);   
config(8).JointPosition = deg2rad(0);  
config(9).JointPosition = deg2rad(45);  
config(10).JointPosition = deg2rad(0); 
config(11).JointPosition = deg2rad(135);   
config(12).JointPosition = deg2rad(0);  
config(13).JointPosition = deg2rad(0); 
config(14).JointPosition = deg2rad(-45);  
config(15).JointPosition = deg2rad(0);  
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

%% CRAWL GAIT (2)

X = 0.12; % far out from leg
S = -0.11; % far down from the leg
A = 0.04; % amplitude of curve
T = 0.10; % width of curve
x_offset = -0.02;
y_offset = 0.05;

T_STALL = 2;
NUM_DATA_POINTS = 16;   
quarter = NUM_DATA_POINTS / 4;

P1 = [-T/2, S];
P2 = [0, S + 2*A];
P3 = [T/2, S];

t = linspace(0, 1, quarter);
xyz = zeros(NUM_DATA_POINTS + 2*T_STALL, 3); 

theta1_min = deg2rad(-45); theta1_max = deg2rad(45);
theta2_min = deg2rad(-90); theta2_max = deg2rad(90);
theta4_min = deg2rad(-90); theta4_max = deg2rad(90);

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

% rotation for rf leg
Rf = [cos(deg2rad(-45)) -sin(deg2rad(-45)) 0; sin(deg2rad(-45)) cos(deg2rad(-45)) 0; 0 0 1];
xyzRf = zeros(NUM_DATA_POINTS + 2*T_STALL, 3);
for i = 1:length(xyz)
    xyzRf(i, :) = Rf*[xyz(i, 1) + x_offset;xyz(i, 2) + y_offset;xyz(i, 3)];
end

% rotation for rb leg
Rb = [cos(deg2rad(45)) -sin(deg2rad(45)) 0; sin(deg2rad(45)) cos(deg2rad(45)) 0; 0 0 1];
xyzRb = zeros(NUM_DATA_POINTS + 2*T_STALL, 3);
for i = 1:length(xyz)
    xyzRb(i, :) = Rb*[xyz(i, 1) + x_offset;xyz(i, 2) - y_offset;xyz(i, 3)]; 
end

% rotation for lb leg
Lb = [cos(deg2rad(-45)) -sin(deg2rad(-45)) 0; sin(deg2rad(-45)) cos(deg2rad(-45)) 0; 0 0 1];
xyzLb = zeros(NUM_DATA_POINTS + 2*T_STALL, 3);
for i = 1:length(xyz)
    xyzLb(i, :) = Lb*[xyz(i, 1) + x_offset;-xyz(i, 2) + y_offset;xyz(i, 3)]; % note that the negative of xyz(i,2) is used because the left legs need to move in the opposite direction with respect to their y axis
end

% rotation for lf leg
Lf = [cos(deg2rad(45)) -sin(deg2rad(45)) 0; sin(deg2rad(45)) cos(deg2rad(45)) 0; 0 0 1];
xyzLf = zeros(NUM_DATA_POINTS + 2*T_STALL, 3);
for i = 1:length(xyz)
    xyzLf(i, :) = Lf*[xyz(i, 1) + x_offset;-xyz(i, 2) - y_offset;xyz(i, 3)]; 
end

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

% leg base marker
scatter3(ax, 0, 0, 0, 80, 'y', 'filled');
text(0,0,0,'Robot Base','FontWeight','bold','Color','b');

% Start with home config display
axbody = show(body, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
title(ax, '3DOF Robot Following Bezier Trajectory');

% 1 indicates swing, 0 indicates stance
rf_phase = [1 0 0 0];
rb_phase = [0 0 1 0];
lb_phase = [0 1 0 0];
lf_phase = [0 0 0 1];

% shift the swing and stance points based on schedule
for phase_var = 0:3
    if rf_phase(phase_var+1) == 1
        xyzRf = [xyzRf(NUM_DATA_POINTS + 2*T_STALL - quarter*phase_var + 1: NUM_DATA_POINTS  + 2*T_STALL, :) xyzRf(1: NUM_DATA_POINTS + 2*T_STALL - quarter*phase_var, :)];
        disp(size(xyzRf))
    elseif lb_phase(phase_var+1) == 1
        xyzLb = [xyzLb(NUM_DATA_POINTS  + 2*T_STALL - quarter*phase_var + 1: NUM_DATA_POINTS + 2*T_STALL, :); xyzLb(1: NUM_DATA_POINTS  + 2*T_STALL - quarter*phase_var, :)];

    elseif rb_phase(phase_var+1) == 1
        xyzRb = [xyzRb(NUM_DATA_POINTS + 2*T_STALL - quarter*phase_var + 1: NUM_DATA_POINTS + 2*T_STALL, :); xyzRb(1: NUM_DATA_POINTS + 2*T_STALL - quarter*phase_var, :)];
    
    elseif lf_phase(phase_var+1) == 1
        xyzLf = [xyzLf(NUM_DATA_POINTS + 2*T_STALL - quarter*phase_var + 1: NUM_DATA_POINTS + 2*T_STALL, :); xyzLf(1: NUM_DATA_POINTS + 2*T_STALL - quarter*phase_var, :)];
    end
end

while true
    for i = 1:NUM_DATA_POINTS + 2*T_STALL
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
        fprintf("Front Right Step: %d, Thetas: %.2f, %.2f, %.2f\n", i-1, rad2deg(theta1Rf), rad2deg(theta2Rf), rad2deg(theta4Rf));
        
        % BACK RIGHT LEG
        x = xyzRb(i,1);
        y = xyzRb(i,2);
        z = xyzRb(i,3);
    
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
        fprintf("Back Left Step: %d, Thetas: %.2f, %.2f, %.2f\n", i-1, rad2deg(theta1Lb), rad2deg(theta2Lb), rad2deg(theta4Lb));

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
            
            % make one leg follow trajectory and rest remain still, doesnt work
            % if i == 1             
            %     config(11).JointPosition = deg2rad(135);   
            %     config(12).JointPosition = deg2rad(0);  
            %     config(13).JointPosition = deg2rad(0); 
            %     config(14).JointPosition = deg2rad(-45);  
            %     config(15).JointPosition = deg2rad(0);  
            %     config(16).JointPosition = deg2rad(-135);  
            %     config(17).JointPosition = deg2rad(0); 
            %     config(18).JointPosition = deg2rad(0);  
            %     config(19).JointPosition = deg2rad(-45);   
            %     config(20).JointPosition = deg2rad(0);
            % else 
            %     config(11).JointPosition = config(11).JointPosition;   
            %     config(12).JointPosition = config(12).JointPosition;  
            %     config(13).JointPosition = config(13).JointPosition; 
            %     config(14).JointPosition = config(14).JointPosition;   
            %     config(15).JointPosition = config(15).JointPosition;  
            %     config(16).JointPosition = config(16).JointPosition;   
            %     config(17).JointPosition = config(17).JointPosition;  
            %     config(18).JointPosition = config(18).JointPosition;  
            %     config(19).JointPosition = config(19).JointPosition;  
            %     config(20).JointPosition = config(20).JointPosition; 
            % end
            
            % update the robot pose in the same axes
            show(body, config, 'Frames', 'on', 'PreservePlot', false, 'Parent', ax);
        end
            
        linkLines = findobj(axbody, 'Type', 'Line');
        set(linkLines, 'LineWidth', 10, 'Color', 'y');  % yellow, thick links 
        drawnow;
    end
end

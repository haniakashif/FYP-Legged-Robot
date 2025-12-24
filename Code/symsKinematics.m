syms l1 l2 l3 l4 alpha1 alpha2 alpha3 alpha4 d1 d2 d3 d4 theta1 theta2 theta3 theta4

T01 = [cos(theta1) -cos(alpha1)*sin(theta1) sin(alpha1)*sin(theta1) l1*cos(theta1); 
    sin(theta1) cos(alpha1)*cos(theta1) -sin(alpha1)*cos(theta1) l1*sin(theta1); 
    0 sin(alpha1) cos(alpha1) d1; 
    0  0 0 1]

T12 = [cos(theta2) -cos(alpha2)*sin(theta2) sin(alpha2)*sin(theta2) l2*cos(theta2); 
    sin(theta2) cos(alpha2)*cos(theta2) -sin(alpha2)*cos(theta2) l2*sin(theta2); 
    0 sin(alpha2) cos(alpha2) d2; 
    0  0 0 1]

T23 = [cos(theta3) -cos(alpha3)*sin(theta3) sin(alpha3)*sin(theta3) l3*cos(theta3); 
    sin(theta3) cos(alpha3)*cos(theta3) -sin(alpha3)*cos(theta3) l3*sin(theta3); 
    0 sin(alpha3) cos(alpha3) d3; 
    0  0 0 1]

T34 = [cos(theta4) -cos(alpha4)*sin(theta4) sin(alpha4)*sin(theta4) l4*cos(theta4); 
    sin(theta4) cos(alpha4)*cos(theta4) -sin(alpha4)*cos(theta4) l4*sin(theta4); 
    0 sin(alpha4) cos(alpha4) d4; 
    0  0 0 1]

%% FORWARD KINEMATICS
T04 = T01*T12*T23*T34;

% right legs
right = simplify(subs(T04, {alpha1, alpha2, alpha3, alpha4, d1, d2, d3, d4}, {pi/2, pi, 0, 0, 0, 0, 0, 0}));
x_r = right(1, 4);
y_r = right(2, 4);
z_r = right(3, 4);
FK_r = [x_r; y_r; z_r]

% left legs
left = simplify(subs(T04, {alpha1, alpha2, alpha3, alpha4, d1, d2, d3, d4}, {-pi/2, pi, 0, 0, 0, 0, 0, 0}));
x_l = left(1, 4);
y_l = left(2, 4);
z_l = left(3, 4);
FK_l = [x_l; y_l ; z_l]

% paper
% paper = subs(T03, {alpha1, alpha2, alpha3, d1, d2, d3}, {pi/2, 0, 0, 0, 0, 0})
% x_p = paper(1, 4);
% y_p = paper(2, 4);
% z_p = paper(3, 4);
% FK_p = [x_p; y_p; z_p]

%% INVERSE KINEMATICS
syms temp x y z r11 r12 r13 r21 r22 r23 r31 r32 r33

T04_placeholder = [r11 r12 r13 x; r21 r22 r23 y; r31 r32 r33 z; 0 0 0 1];
T14_placeholder = T01\T04_placeholder;
T14 = T01\T04;

% right
LHS_right_all = simplify(subs(T14_placeholder, {alpha1, alpha2, alpha3, alpha4, d1, d2, d3, d4}, {pi/2, pi, 0, 0, 0, 0, 0, 0}))
RHS_right_all = simplify(subs(T14, {alpha1, alpha2, alpha3, alpha4, d1, d2, d3, d4}, {pi/2, pi, 0, 0, 0, 0, 0, 0}))
LHS_right = LHS_right_all(1:3,4)
RHS_right = RHS_right_all(1:3,4)

% left
LHS_left_all = simplify(subs(T14_placeholder, {alpha1, alpha2, alpha3, alpha4, d1, d2, d3, d4}, {-pi/2, pi, 0, 0, 0, 0, 0, 0}));
RHS_left_all = simplify(subs(T14, {alpha1, alpha2, alpha3, alpha4, d1, d2, d3, d4}, {-pi/2, pi, 0, 0, 0, 0, 0, 0}));
LHS_left = LHS_left_all(1:3,4)
RHS_left = RHS_left_all(1:3,4)
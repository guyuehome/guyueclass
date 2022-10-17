% clear all existing variables in the workspace
clear;
% dynamics params(inertia1 inertia2 linklength1 linklength2 mass1 mass2 lengthCOM1 lenthCOM2 gravityacc)
syms I1 I2 l1 l2 m1 m2 a1 a2 g real
% control input(jonttorque1 jointtorque2)
syms u1 u2 real
% states(jointpos1 jointpos2 jointvel1 jointvel2)
syms theta1 theta2 theta1dot theta2dot real
% external force suffered at the end effector frame
syms fx fy real

% I1Val = 1;
% I2Val = 1;
% l1Val = 0.3;
% l2Val = 0.3;
% m1Val = 0.5;
% m2Val = 0.5;
% a1Val = 0.15;
% a2Val = 0.15;
% gVal = 9.81;
% paramValues = [I1Val I2Val l1Val l2Val m1Val m2Val a1Val a2Val gVal];
%% --- Constructing Lagrange ---
% Kinematic Energy
Ek = 1/2*m1*a1^2*theta1dot^2 + 1/2*I1*theta1dot^2 ...
           + 1/2*m2*(l1^2*theta1dot^2 + a2^2*(theta1dot + theta2dot)^2 + 2*l1*a2*cos(theta2)*theta1dot*(theta1dot+theta2dot))...
           + 1/2*I2*(theta1dot + theta2dot)^2;
% Potential Energy
Ep = m1*g*a1*cos(theta1) + m2*g*(l1*cos(theta1) + a2*cos(theta1 + theta2));
% Lagrange function
Lagrange = Ek - Ep;
%% --- Obtaining Dynamics Terms by Lagrangian Dynamics ---     
% M -- joint space mass-inertia mztrix
% c -- coriolis centrifugal torque
% C -- coriolis centrifugal matrix
% G -- gravity torque
% B -- c+G
pEk_ptheta1dot = jacobian(Ek,theta1dot);
M11 = jacobian(pEk_ptheta1dot,theta1dot);
M12 = jacobian(pEk_ptheta1dot,theta2dot);

M111 = jacobian(M11,theta1); M112 = jacobian(M11,theta2); M121 = jacobian(M12,theta1); M122 = jacobian(M12,theta2);
c1 =  jacobian(pEk_ptheta1dot,theta1)*theta1dot + jacobian(pEk_ptheta1dot,theta2)*theta2dot - jacobian(Ek,theta1);
G1 =  jacobian(Ep,theta1);
B1 =  c1 + G1;

dEk_dtheta2dot = jacobian(Lagrange,theta2dot);
M21 = M12; % symmetric matrix
M22 = jacobian(dEk_dtheta2dot,theta2dot);
M221 = jacobian(M22,theta1); M222 = jacobian(M22,theta2);
c2 =  jacobian(dEk_dtheta2dot,theta1)*theta1dot + jacobian(dEk_dtheta2dot,theta2)*theta2dot - jacobian(Ek,theta2);
G2 =  jacobian(Ep,theta2);
B2 =  c2 + G2;

C11 = 1/2*M111*theta1dot + 1/2*M112*theta2dot;
C12 = 1/2*M112*theta1dot + (M122 - 1/2*M221)*theta2dot;
C21 = (M121-1/2*M112)*theta1dot + 1/2*M221*theta2dot;
C22 = 1/2*M221*theta1dot + 1/2*M222*theta2dot;

M = [M11 M12;M21 M22];
M = simplify(M);
C = [C11,C12;C21,C22];
C = simplify(C);
c = [c1;c2];
c = simplify(c);
G = [G1;G2];
G = simplify(G);
B = [B1;B2];
B = simplify(B);

% velidate c = C*qdot
simplify(c - C*[theta1dot;theta2dot])
% validate M_dot = C + C^T
M_vec = [M11;M21;M12;M22];
M_dot  = jacobian(M_vec,theta1)*theta1dot + jacobian(M_vec,theta2)*theta2dot;
M_dot = reshape(M_dot,2,2);
simplify(M_dot - C - C.')

control = [u1;u2];

%% --- direct kinematics ---
xee = l1*sin(theta1) + l2*sin(theta1+theta2);
yee = l1*cos(theta1) + l2*cos(theta1+theta2);

%% --- geometric jacobian ---
J11 = jacobian(xee,theta1);
J12 = jacobian(xee,theta2);
J21 = jacobian(yee,theta1);
J22 = jacobian(yee,theta2);
J = [J11,J12;J21,J22];
J = simplify(J);

%% --- end effector velocity ---
xeedot = J(1,:)*[theta1dot;theta2dot];
yeedot = J(2,:)*[theta1dot;theta2dot];

%% --- derivative of geometric jacobian respect to time ---
J_vec = reshape(J,4,1);
dJ_dtheta = jacobian(J_vec,[theta1,theta2]);%*theta1dot + jacobian(J,theta2)*theta2dot;
dJ_dt = dJ_dtheta*[theta1dot;theta2dot];
dJ_dt = reshape(dJ_dt,2,2);
dJ_dt = simplify(dJ_dt);

%% --- dynamics model ---
f(1) = theta1dot;
f(2) = theta2dot;
f(3:4) = M\(-B + control + J.'*[fx;fy]);
f = simplify(f);

%% --- generate matlab files for numerical computation ---
mkdir generated;
cd generated;
matlabFunction(transpose(f),'File','Forward_Dynamics',...
    'Vars',{transpose([theta1 theta2 theta1dot theta2dot]),control,[fx;fy],transpose([I1 I2 l1 l2 m1 m2 a1 a2 g])});
matlabFunction(M,'File','Mass_Matrix',...
    'Vars',{transpose([theta1 theta2]),transpose([I1 I2 l1 l2 m1 m2 a1 a2 g])});
matlabFunction(C,'File','Coriolis_Centrifugal_Matrix',...
    'Vars',{transpose([theta1 theta2 theta1dot theta2dot]),transpose([l1 l2 m1 m2 a1 a2 g])});
matlabFunction(c,'File','Coriolis_Centrifugal_Torque',...
    'Vars',{transpose([theta1 theta2 theta1dot theta2dot]),transpose([l1 l2 m1 m2 a1 a2 g])});
matlabFunction(G,'File','Gravity_Torque',...
    'Vars',{transpose([theta1 theta2]),transpose([l1 l2 m1 m2 a1 a2 g])});
matlabFunction([xee;yee],'File','Direct_Kinematics',...
    'Vars',{transpose([theta1 theta2]),transpose([l1 l2])});
matlabFunction(J,'File','Geometric_Jacobian',...
    'Vars',{transpose([theta1 theta2]),transpose([l1 l2])});
matlabFunction(dJ_dt,'File','Geometric_Jacobian_Derivative',...
    'Vars',{transpose([theta1 theta2 theta1dot theta2dot]),transpose([l1 l2])});

%% --- impedance controller ---
syms kpx kpy kdx kdy xd yd xdotd ydotd real
tau_impedance = J.'*([kpx,0;0,kpy]*([xd;yd]-[xee;yee]) + [kdx,0;0,kdy]*([xdotd;ydotd]-[xeedot;yeedot]));
matlabFunction(tau_impedance,'File','Impedance_Controller',...
    'Vars',{transpose([theta1 theta2 theta1dot theta2dot]),transpose([l1 l2]),[kpx;kpy;kdx;kdy],[xd;yd],[xdotd;ydotd]});

%% --- admittance controller ---
syms mx my real
tau_admittance = J.'*([kpx,0;0,kpy]*([xd;yd]-[xee;yee]) + [kdx,0;0,kdy]*([xdotd;ydotd]-[xeedot;yeedot]) + [fx;fy]);
acc_admittance = M\tau_admittance;%[mx,0;0,my]
matlabFunction(acc_admittance,'File','Admittance_Controller',...
    'Vars',{transpose([theta1 theta2 theta1dot theta2dot]),[fx;fy],transpose([I1 I2 l1 l2 m1 m2 a1 a2 g]),[mx;my;kpx;kpy;kdx;kdy],[xd;yd],[xdotd;ydotd]});

%% --- change directory and add path ---
cd ..
addpath('./generated');
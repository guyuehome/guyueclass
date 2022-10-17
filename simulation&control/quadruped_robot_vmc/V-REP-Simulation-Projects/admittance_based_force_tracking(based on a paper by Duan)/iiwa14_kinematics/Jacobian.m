function [J, A_mat_products] = Jacobian(joints,robot_type)
% JACOBIAN
% Calculates Jacobian of iiwa

% Input:  joints - joint positions
%         robot_type - iiwa7(0) or iiwa14(~0)
% Output: J - Jacobian

Origins = zeros(3,7);
J_ang = zeros(3,7);
J_trans = zeros(3,7);

A_mats = cell(1,7);
A_mat_products = cell(1,7);

%Robot parameters
%Link length
l = zeros(1,4);
if robot_type == 0
    l = [0.34 0.4 0.4 0.126]; % iiwa7
else
%     disp("iiwa14");
    l = [0.36 0.42 0.4 0.126]; % iiwa14
end
%Denavit-Hartenberg parameters 7 DoF
%DH: [a, alpha,    d, theta]
dh = [0	 -pi/2	 l(1)    0;
    0	  pi/2      0    0;
    0	  pi/2   l(2)	 0;
    0  -pi/2      0    0;
    0  -pi/2   l(3)    0;
    0   pi/2      0    0;
    0      0   l(4)    0];

for i=1:7
    A_mats{i} = dh_calc(dh(i,1),dh(i,2),dh(i,3),joints(i));
end

A_mat_products{1} = A_mats{1};
for i=2:7
    A_mat_products{i} = A_mat_products{i-1}*A_mats{i};
end

A = A_mat_products{7};
wvec = A(1:3,4);

zvec = [0;0;1];
J_ang(:,1) = zvec;
oi = [0;0;0];
Origins(:,1) = oi;

for i=2:7
    A = A_mat_products{i-1};
    zvec = A(1:3,3);
    J_ang(:,i) = zvec;
    oi = A(1:3,4);
    Origins(:,i) = oi;
end

for i=1:7
    zvec = J_ang(:,i);
    oi = Origins(:,i);
    rvec = wvec - oi;
    J_trans(:,i) = cross(zvec,rvec);
end

J = [J_trans;J_ang];
end
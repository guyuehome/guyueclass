function [ joints, s_mat, w_mat ] = InverseKinematics( pose, nsparam, rconf, robot_type )
% INVERSEKINEMATICS
%
% Calculates the inverse kinematics for a KUKA LBR iiwa manipulator.
%
% Input:  pose    - Homogeneous Matrix size(4,4)
%         nsparam - Arm Angle
%         rconf   - Robot Configuration 8-bit number
% Output: joints  - Joint values size(1,7).
%         s_mat   - Shoulder joint matrices As, Bs and Cs
%         w_mat   - Wrist joint matrices Aw, Bw and Cw
%         robot_type - iiwa7 (0) or iiwa14 (~0)

[ arm, elbow, wrist ] = Configuration(rconf);

%Tolerance
tol = 1e-8;

%Robot parameters
%Link length
l = zeros(1,6);
if robot_type == 0
    l = [0.34 0.4 0.4 0.126]; % iiwa7
else
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

%Number of joints
nj = size(dh,1);

% Joint values of virtual manipulator
joints = zeros(1,nj);

% Shoulder rotation matrices
s_mat = zeros(3,3,3);

% Wrist rotation matrices
w_mat = zeros(3,3,3);

xend = pose(1:3,4); % end-effector position from base    
xs = [0 0 dh(1,3)]'; % shoulder position from base 
xwt = [0 0 dh(end,3)]'; % end-effector position from wrist
xw = xend - pose(1:3,1:3)*xwt; % wrist position from base
xsw = xw - xs; % shoulder to wrist vector
usw = unit(xsw);

lbs = l(1);
lse = l(2); % upper arm length (shoulder to elbow)
lew = l(3); % lower arm length (elbow to wrist)

%Check if pose is within arm+forearm reach
assert(norm(xsw) < lse + lew && norm(xsw) > lse - lew, 'Specified pose outside reacheable workspace');

% -- Joint 4 --
% Elbow joint can be directly calculated since it does only depend on the 
% robot configuration and the xsw vector 
assert(abs((norm(xsw)^2 - lse^2 - lew^2)-(2*lse*lew)) > tol, 'Elbow singularity. Tip at reach limit.');
% Cosine law - According to our robot, joint 4 rotates backwards
joints(4) = elbow * acos((norm(xsw)^2 - lse^2 - lew^2)/(2*lse*lew));

%Added 

T34 = dh_calc(dh(4,1),dh(4,2),dh(4,3),joints(4));
R34 = T34(1:3,1:3);

% Shoulder Joints
% First compute the reference joint angles when the arm angle is zero.
[~, R03_o, ~] = ReferencePlane(pose, elbow, robot_type);

skew_usw = skew(usw);
% Following eq. (15), the auxiliary matrixes As Bs and Cs can be calculated 
% by substituting eq. (6) into (9). 
% R0psi = I3 + sin(psi)*skew_usw + (1-cos(psi))*skew_usw²    (6)
% R03 = R0psi * R03_o                                         (9)
% Substituting (distributive prop.) we get:
% R03 = R03_o*skew_usw*sin(psi) + R03_o*(-skew_usw²)*cos(psi) + R03_o(I3 + skew_usw²)
% R03 =      As       *sin(psi) +        Bs         *cos(psi) +          Cs
As = skew_usw * R03_o;
Bs = -skew_usw^2 * R03_o;
Cs = (usw*usw') * R03_o;

psi = nsparam;
R03 = As*sin(psi) + Bs*cos(psi) + Cs;

% T03 transformation matrix (DH parameters)
%[ cos(j1)*cos(j2)*cos(j3) - sin(j1)*sin(j3), cos(j1)*sin(j2), cos(j3)*sin(j1) + cos(j1)*cos(j2)*sin(j3), 0.4*cos(j1)*sin(j2)]
%[ cos(j1)*sin(j3) + cos(j2)*cos(j3)*sin(j1), sin(j1)*sin(j2), cos(j2)*sin(j1)*sin(j3) - cos(j1)*cos(j3), 0.4*sin(j1)*sin(j2)]
%[                          -cos(j3)*sin(j2),         cos(j2),                          -sin(j2)*sin(j3),  0.4*cos(j2) + 0.34]
%[                                         0,               0,                                         0,                   1]
joints(1) = atan2(arm * R03(2,2), arm * R03(1,2));
joints(2) = arm * acos(R03(3,2));

joints(3) = atan2(arm * -R03(3,3), arm * -R03(3,1));

Aw = R34' * As' * pose(1:3,1:3);
Bw = R34' * Bs' * pose(1:3,1:3);
Cw = R34' * Cs' * pose(1:3,1:3);

R47 = Aw*sin(psi) + Bw*cos(psi) + Cw;

% T47 transformation matrix (DH parameters)
%[ cos(j5)*cos(j6)*cos(j7) - sin(j5)*sin(j7), - cos(j7)*sin(j5) - cos(j5)*cos(j6)*sin(j7), cos(j5)*sin(j6), (63*cos(j5)*sin(j6))/500]
%[ cos(j5)*sin(j7) + cos(j6)*cos(j7)*sin(j5),   cos(j5)*cos(j7) - cos(j6)*sin(j5)*sin(j7), sin(j5)*sin(j6), (63*sin(j5)*sin(j6))/500]
%[                          -cos(j7)*sin(j6),                             sin(j6)*sin(j7),         cos(j6),   (63*cos(j6))/500 + 2/5]
%[                                         0,                                           0,               0,                        1]
joints(5) = atan2(wrist * R47(2,3), wrist * R47(1,3));
joints(6) = wrist * acos(R47(3,3));
joints(7) = atan2(wrist * R47(3,2), wrist * -R47(3,1));

% Grouping Shoulder and Wrist matrices that will be used by the joint
% limit algorithms
s_mat(:,:,1) = As; 
s_mat(:,:,2) = Bs;
s_mat(:,:,3) = Cs;
w_mat(:,:,1) = Aw;
w_mat(:,:,2) = Bw;
w_mat(:,:,3) = Cw;

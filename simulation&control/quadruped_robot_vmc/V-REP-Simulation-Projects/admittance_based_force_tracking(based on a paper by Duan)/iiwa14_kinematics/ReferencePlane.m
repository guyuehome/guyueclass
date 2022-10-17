function [ ref_plan_vector, rot_base_elbow, joints ] = ReferencePlane( pose, elbow, robot_type)
% REFERENCEPLANE
% 
% Calculates the vector normal to the reference plane.
% A virtual robotic manipulator is created with the same structure as the
% real robot, but keeping its joint3 = 0.
%
% From the current end-effector position and robot configuration, it
% computes the set of joint positions for the virtual manipulator (i.e. theta_3=0)
% With this we compute the plane of the virtual Shoulder-Elbow-Wrist, which
% is the reference plane. The NSParam is calculated from the angle between
% the actual robot Shoulder-Elbow-Wrist plane and the reference plane.
%
% Input:  pose            - Homogeneous Matrix size(4,4) of current robot pose
%         rconf           - Robot Configuration 8-bit number
% Output: ref_plan_vector - Vector normal to the reference plane.
%         rot_base_elbow  - Rotation Matrix size(3,3) from base to elbow
%         joints          - Joint values of the virtual robot (debug)
%          robot_type - iiwa7 (0) or iiwa14 (~0)
%
% The robot configuration parameter is considered because there is usually
% 2 possible solutions: elbow 'up' and 'down'. Therefore and as the author
% suggested we are selecting the configuration that matches the current
% robot.

% Calculations tolerance
tol = 1e-6;

%Robot parameters
%Link length
l = zeros(1,6);
if ~robot_type
    l = [0.34 0.4 0.4 0.126]; % iiwa7
else
    l = [0.36 0.42 0.4 0.126]; % iiwa14
end

%Denavit-Hartenberg parameters 7 DoF
%DH: [a, alpha,    d, theta] 
dh = [0	 -pi/2	 l(1)   0;
      0	  pi/2      0   0;
      0	  pi/2   l(2)	0; %theta3 == 0
      0  -pi/2      0   0;
      0  -pi/2   l(3)   0;
      0   pi/2      0   0;
      0      0   l(4)   0];

% Joint values of virtual manipulator
joints = zeros(1,7);

xend = pose(1:3,4); % end-effector position from base    
xs0 = [0 0 dh(1,3)]'; % shoulder position from base 
xwt = [0 0 dh(end,3)]'; % end-effector position from wrist
xw0 = xend - pose(1:3,1:3)*xwt; % wrist position from base
xsw = xw0 - xs0; % shoulder to wrist vector

lbs = l(1);
lse = l(2); % upper arm length (shoulder to elbow)
lew = l(3); % lower arm length (elbow to wrist)

%Check if pose is within arm+forearm reach
assert(norm(xsw) < lse + lew && norm(xsw) > lse - lew, 'Specified pose outside reacheable workspace');

% -- Joint 4 --
% Elbow joint can be directly calculated since it does only depend on the 
% robot configuration and the xsw vector 
assert(abs((norm(xsw)^2 - lse^2 - lew^2)-(2*lse*lew)) > tol, 'Elbow singularity. Tip at reach limit.');
% Cosine law
joints(4) = elbow * acos((norm(xsw)^2 - lse^2 - lew^2)/(2*lse*lew));

% Shoulder Joints
T34 = dh_calc(dh(4,1),dh(4,2),dh(4,3),joints(4));
R34 = T34(1:3,1:3);

% These are the vectors corresponding to our DH parameters
xse = [0 lse 0]'; 
xew = [0 0 lew]'; 
% m = member between parentisis. Check equation (14)
m = xse + R34*xew; 

% -- Joint 1 --
% Since joint3 is locked as 0, the only joint to define the orientation of
% the xsw vector in the xy-plane is joint 1. Therefore and since we are
% only interested in the transformation T03 (disregarding joint limits), we
% chose to simply set joint 1 as the atan of xsw y and x coordinates 
% (even if if goes beyond the joint limit).

%Cannot be this because if x and y are 0, then it is not defined.
if(norm(cross(xsw, [0 0 1])) > tol)
    joints(1) = atan2(xsw(2),xsw(1));
else 
    joints(1) = 0;
end

% -- Joint 2 --
% Can be found through geometric relations
% Let phi be the angle E-S-W, and theta2 the angle (z-axis)-S-E.
% Then, theta2 = atan2(r,xsw(3)) -/+ phi.
% phi can be calculated, as a function of theta3:
%   atan2(lew*sin(theta4),lse+lew*cos(theta4))
% z-axis
%   ^
%   |  E O------------O W
%   |   /        .  
%   |  /      .
%   | /    .    xsw
%   |/  .
% S O___________________ r-axis
%
r = hypot(xsw(1), xsw(2));
dsw = norm(xsw);
phi = acos((lse^2+dsw^2-lew^2)/(2*lse*dsw));

joints(2) = atan2(r, xsw(3)) + elbow * phi;

% Lower arm transformation
T01 = dh_calc(dh(1,1),dh(1,2),dh(1,3),joints(1));
T12 = dh_calc(dh(2,1),dh(2,2),dh(2,3),joints(2));
T23 = dh_calc(dh(3,1),dh(3,2),dh(3,3),0);
T34 = dh_calc(dh(4,1),dh(4,2),dh(4,3),joints(4));
T04 = T01*T12*T23*T34;

rot_base_elbow = T01(1:3,1:3)*T12(1:3,1:3)*T23(1:3,1:3);

% With T03 we can calculate the reference elbow position and with it the
% vector normal to the reference plane.
x0e = T04(1:3,4); % reference elbow position
v1 = unit(x0e - xs0); % unit vector from shoulder to elbow
v2 = unit(xw0 - xs0); % unit vector from shoulder to wrist

ref_plan_vector = cross(v1,v2);

end
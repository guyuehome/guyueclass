function [ pose, nsparam, rconf, jout ] = ForwardKinematics( joints, robot_type )
% FORWARDKINEMATICS
%
% Calculates the forward kinematics for a KUKA LBR iiwa manipulator.
%
% Input:    joints  - Joint values size(1,7).
% Output:   pose    - Homogeneous Matrix size(4,4)
%           nsparam - Arm Angle
%           rconf   - Robot Configuration 8-bit number
%           robot_type - iiwa7 (0) or iiwa14 (~0)
%Tolerance
tol = 1e-8;

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

%Number of joints
nj = size(dh,1);

%Robot configuration
rconf = (joints(2) < 0) + 2*(joints(4) < 0) + 4*(joints(6) < 0);

[ arm, elbow, wrist ] = Configuration( rconf );

%Assign joint values to the theta column of the DH parameters
dh(:,4) = joints;
%Store transformations from the base reference frame to the index joint
% e.g: tr(:,:,2) is the T02 -> transformation from base to joint 2 (DH table)
tr = zeros(4,4,nj);
%Rotation Matrix applied with Denavit-Hartenberg parameters [same as (3)]
%R = [Xx,Yx,Zx,   --  Xx = cos(theta), Yx = -sin(theta) * cos(alpha), Zx =  sin(theta) * sin(alpha)
%     Xy,YY,Zy,   --  Xy = sin(theta), Yy =  cos(theta) * cos(alpha), Zy = -cos(theta) * sin(alpha)
%     Xz,Yz,Zz];  --  Xz = 0.0,        Yz =  sin(alpha),              Zz =  cos(alpha)

for i=1:nj
    a = dh(i,1); alpha = dh(i,2); d = dh(i,3); theta = dh(i,4);
    v = [ a*cos(theta), a*sin(theta), d ];
    Xx = cos(theta); Yx = -sin(theta) * cos(alpha); Zx =  sin(theta) * sin(alpha);
    Xy = sin(theta); Yy =  cos(theta) * cos(alpha); Zy = -cos(theta) * sin(alpha);
    Xz = 0.0;        Yz =  sin(alpha);  Zz =  cos(alpha);
    tmp = [Xx, Yx, Zx, v(1);
        Xy, Yy, Zy, v(2);
        Xz, Yz, Zz, v(3);
        0,  0,  0,  1];
    if(i==1)
        tr(:,:,1) = tmp;
    else
        tr(:,:,i) = tr(:,:,i-1) * tmp;
    end
end

xs = tr(1:3,4,1);  % shoulder position from base
xe = tr(1:3,4,4);  % elbow position from base
xw = tr(1:3,4,6);  % wrist position from base
xsw = xw-xs;       % wrist position from shoulder

pose = tr(:,:,end); %end-effector transformation from base

%Calculate the nsparam - Arm Angle
% The arm plane is defined by the plane formed by the xs, xe and xw points
% The reference plane is defined by the xs and xw and a xe0 (explained below)
% . A virtual robotic manipulator is created from the KUKA LBR iiwa manipulator
%   structure. Equal in everything least the 3rd joint, which is fixed as 0.
% . Now we compute the Inverse Kinematics to place the virtual robot in the
%   same pose as the real robot.
% . The elbow position of this virtual robot is xe0. Thus, with xs, xw and
%   xe0 we form the reference plane
[vv, ~, jout] = ReferencePlane(pose, elbow, robot_type);

% vv is the vector normal to the reference plane: xs-xe0-xw
% vc is the vector normal to the current plane:   xs-xe-xw
v1 = unit(xe-xs);
v2 = unit(xw-xs);
vc = cross(v1,v2);

cos_ns = dot(unit(vv),unit(vc));
if(abs(norm(cos_ns))>1)
    cos_ns = sign(cos_ns);
end
%this vector will give the sign of the nsparam
v3 = cross(unit(vv),unit(vc));
if(norm(v3) > tol)
    nsparam = sign(dot(v3,xsw)) * acos(cos_ns);
else
    if(norm(vv - vc) < tol)
        nsparam = 0;
    else
        nsparam = pi;
    end
end
end


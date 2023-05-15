function [dXin] = vdn_one_rod(Xin,u,i)
% model
global rod
phi1        = Xin(1,1);
theta1       = Xin(2,1);
psi1        = Xin(3,1);
dphi1       = Xin(4,1);
dtheta1      = Xin(5,1);
dpsi1       = Xin(6,1);
qi          = Xin(1:3,:);
dqi         = Xin(4:6,:);

gamma1      = [dphi1;dtheta1;dpsi1];

rot11   = [cos(phi1),-sin(phi1),0;sin(phi1),cos(phi1),0;0,0,1];
rot12   = [1,0,0;0,cos(theta1),-sin(theta1);0,sin(theta1),cos(theta1)];
rot13   = [cos(psi1),0,sin(psi1);0,1,0;-sin(psi1),0,cos(psi1)];
A1      = rot11*rot12*rot13;
G1_     = [-sin(psi1)*cos(theta1),cos(psi1),0;sin(theta1),0,1;cos(psi1)*cos(theta1),sin(psi1),0];
w1_     = G1_*gamma1;
dA1     = A1*skew(w1_);
dG1_    = [-cos(psi1).*cos(theta1).*dpsi1+sin(psi1).*sin(theta1).*dtheta1,-sin(psi1).*dpsi1,0;
            cos(theta1).*dtheta1                                      ,0              ,0;
           -sin(psi1).*cos(theta1).*dpsi1-cos(psi1).*sin(theta1).*dtheta1,cos(psi1).*dpsi1,0];
%------------------------------ Qe Qv
Qe  = [rod.m1*[0;0;rod.g];[0;0;0]]+u;
Qv  = [[0;0;0];cross(w1_,(rod.J1*w1_))];
%------------------------------ B dB
B               = [A1*skew(rod.u11)*G1_;G1_];
dB              = [dA1*skew(rod.u11)*G1_+A1*skew(rod.u11)*dG1_;dG1_];

equ_left        = B.'*rod.M*B;
equ_right       = B.'*(Qe+Qv-rod.M*dB*dqi);
ddqi            = equ_left\equ_right;

dXin    = [dqi;ddqi];
end
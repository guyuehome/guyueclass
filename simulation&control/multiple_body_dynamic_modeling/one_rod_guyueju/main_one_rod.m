function main_one_rod
close all;
clear;
clc;
% parameters
global rod
rod.l1  = 1;
rod.m1  = diag(repmat(10,1,3));
rod.J1  = diag([1,1,1]);
rod.u11 = [-rod.l1/2;0;0];
rod.u12 = [rod.l1/2;0;0];
rod.I1  = [0;1];
rod.M   = blkdiag(rod.m1,rod.J1);
rod.g   = -9.8;
% intial state and input
t_all       = 15;           % 仿真总时间
T           = 0.05;        % 仿真步长
N           = t_all/T;
phi1        = 0;
theta1      = 0;
psi1        = 0;
dphi1       = 0;
dtheta1     = 0;
dpsi1       = 0;
Xin         = [phi1;theta1;psi1;dphi1;dtheta1;dpsi1];
Xin         = Xin/180*pi;
Fe_Me       = zeros(6,N+1);
Fe_Me(2,1:10)   = 10;

rod.Xlog        = zeros(6,N+1);
rod.Xlog(:,1)   = Xin;
rod.tlog        = zeros(N+1,1);
rod.point       = zeros(3,N);
% main
figure(1)
for i=1:N
    phi1        = Xin(1,1);
    theta1      = Xin(2,1);
    psi1        = Xin(3,1);
    rot11   = [cos(phi1),-sin(phi1),0;sin(phi1),cos(phi1),0;0,0,1];
    rot12   = [1,0,0;0,cos(theta1),-sin(theta1);0,sin(theta1),cos(theta1)];
    rot13   = [cos(psi1),0,sin(psi1);0,1,0;-sin(psi1),0,cos(psi1)];
    A1      = rot11*rot12*rot13;
    R1      = -A1*rod.u11;

    r11 = [0;0;0];
    r12 = R1+A1*rod.u12;
    rod.point1(:,i) = r12;
    
    plot3([r11(1,1);r12(1,1)],[r11(2,1);r12(2,1)],[r11(3,1);r12(3,1)],'Color','k','LineWidth',1)
    hold on
    plot3(rod.point1(1,1:i),rod.point1(2,1:i),rod.point1(3,1:i),'Color','b','LineWidth',1)
    patch([1 1 -1 -1],[1 -1 -1 1],[0 0 0 0],'g','FaceAlpha',0.1)
    title(sprintf('$$t: %0.3f$$',rod.tlog(i,1)),'Interpreter','latex');
    hold off
    
    xlim([-1,1]);ylim([-1,1]);zlim([-1.5,0.5])
%     view(3)
%     view(0,0)
    view(0,90)
    axis equal
    pause(0.005)

    u       = Fe_Me(:,i);
    Xout    = runge_kutta4(@vdn_one_rod,Xin,u,T,i);
    Xin     = Xout;
    rod.Xlog(:,i+1) = Xin;
    rod.tlog(i+1,1)   = i*T;
end
end
function main_euler_angle
clc
close all
clear all
% axis
x0  = [1;0;0];
y0  = [0;1;0];
z0  = [0;0;1];
% simulation parameter
t_all   = 10;
dt      = 0.02;
N       = t_all/dt;
tlog    = zeros(1,N+1);
% state parameter
state       = zeros(9,N+1);
state(:,1)  = [x0;y0;z0];
% control parameter
euler_angle       = zeros(3,N);
% simulation
figure(1)
for i = 1:N
    t                   = i*dt;
    tlog(1,i+1)         = t;
    euler_angle(:,i)    = control_parameter(t);

    phi     = euler_angle(1,i);
    theta   = euler_angle(2,i);
    psi     = euler_angle(3,i);

    A01     = [cos(phi),-sin(phi),0;sin(phi),cos(phi),0;0,0,1];
    A12     = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
    A23     = [cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1];
    A       = A01*A12*A23;

    state(:,i+1)    = [A*state(1:3,1);A*state(4:6,1);A*state(7:9,1)];
    plot_realtime(state,i,t,dt)
end
end
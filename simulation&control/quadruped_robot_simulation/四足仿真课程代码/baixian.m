clc;
clear;
Ts=1;    %周期
xs=-0.1;    %起点x位置
xf=0.1;     %终点x位置
zs=-0.582;   %z起点位置
h=0.1;      %抬腿高度
x=[];
z=[];
for t=0:0.01:Ts
    sigma=2*pi*t/(Ts);
    xep=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs;
    zep=h*(1-cos(sigma))/2+zs;
    x=[x,xep];
    z=[z,zep];
end
plot(x,z,'r','LineWidth',3)
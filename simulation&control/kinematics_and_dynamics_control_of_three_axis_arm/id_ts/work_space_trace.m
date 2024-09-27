function [sys,x0,str,ts] = work_space_trace(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs =9;
sizes.NumInputs = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0  = [];
str = [];
ts  = [-1 0];
function sys=mdlOutputs(t,~,~)


rate = 3;
Ampli = 0.04;
x_0 = 0.15;
y_0 = 0;
z_0 = 0.01;


%螺旋线
x = x_0+Ampli*cos(t*rate); dx=-Ampli*rate*sin(t*rate); ddx=-Ampli*rate*rate*cos(t*rate);

y = y_0+Ampli*sin(t*rate); dy= Ampli*rate*cos(t*rate); ddy=-Ampli*rate*rate*sin(t*rate);

z = z_0+0.015*t;  dz = 0.015; ddz = 0;



%姿态无要求


%单位：m
sys(1)=x;
sys(2)=y;
sys(3)=z;
sys(4)=dx;
sys(5)=dy;
sys(6)=dz;
sys(7)=ddx;
sys(8)=ddy;
sys(9)=ddz;


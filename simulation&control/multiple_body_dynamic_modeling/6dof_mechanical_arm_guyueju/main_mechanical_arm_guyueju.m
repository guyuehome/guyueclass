function main_mechanical_arm_guyueju
clc
close all
clear all
% arm parameter
global robotarm error des_pos
robotarm.n      = 6;%杆件数量
robotarm.dof    = 6;%自由度
robotarm.g      = [0;0;9.8*0];%重力加速度

robotarm.le1    = 0.2;%长度
robotarm.th1    = 0.1;%厚度
robotarm.wi1    = 0.1;%宽度
robotarm.u11    = [-robotarm.le1/2;0;0];%两端铰接点的局部位置矢量
robotarm.u12    = [robotarm.le1/2;0;0];
robotarm.m1     = 1*eye(3,3);%质量
robotarm.j1     = 0.1*eye(3,3);%惯量

robotarm.le2    = 0.5;
robotarm.th2    = 0.1;
robotarm.wi2    = 0.1;
robotarm.u21    = [-robotarm.le2/2;0;0];
robotarm.u22    = [robotarm.le2/2;0;0];
robotarm.m2     = 1*eye(3,3);
robotarm.j2     = 0.1*eye(3,3);

robotarm.le3    = 0.5;
robotarm.th3    = 0.1;
robotarm.wi3    = 0.1;
robotarm.u31    = [-robotarm.le3/2;0;0];
robotarm.u32    = [robotarm.le3/2;0;0];
robotarm.m3     = 1*eye(3,3);
robotarm.j3     = 0.1*eye(3,3);

robotarm.le4    = 0.8;
robotarm.th4    = 0.1;
robotarm.wi4    = 0.1;
robotarm.u41    = [-robotarm.le4/2;0;0];
robotarm.u42    = [robotarm.le4/2;0;0];
robotarm.m4     = 1*eye(3,3);
robotarm.j4     = 0.1*eye(3,3);

robotarm.le5    = 0.2;
robotarm.th5    = 0.1;
robotarm.wi5    = 0.1;
robotarm.u51    = [-robotarm.le5/2;0;0];
robotarm.u52    = [robotarm.le5/2;0;0];
robotarm.m5     = 1*eye(3,3);
robotarm.j5     = 0.1*eye(3,3);

robotarm.le6    = 0.1;
robotarm.th6    = 0.1;
robotarm.wi6    = 0.1;
robotarm.u61    = [-robotarm.le6/2;0;0];
robotarm.u62    = [robotarm.le6/2;0;0];
robotarm.m6     = 1*eye(3,3);
robotarm.j6     = 0.1*eye(3,3);
% simulation parameter
t_all   = 25;
dt      = 0.02;
N       = t_all/dt;
tlog    = zeros(1,N+1);
% state parameter
ini_arm1    = [0.1;0;0.05;0;0;0;0;0;0;0;0;0];%12*1
ini_arm2    = [0.2;0;0.3;-pi/2;0;0;0;0;0;0;0;0];
ini_arm3    = [0.2;0;0.8;-pi/2;0;0;0;0;0;0;0;0];
ini_arm4    = [0.6;0;1.05;0;0;0;0;0;0;0;0;0];
ini_arm5    = [1.1;0;1.05;0;0;0;0;0;0;0;0;0];
ini_arm6    = [1.25;0;1.05;0;0;0;0;0;0;0;0;0];
ini_state   = [ini_arm1;ini_arm2;ini_arm3;ini_arm4;ini_arm5;ini_arm6];%72*1
State       = zeros(72,N+1);
State(:,1)  = ini_state;
% control parameter
error.int_arm1      = 0;
error.int_arm2      = 0;
error.int_arm3      = 0;
error.int_arm4      = 0;
error.int_arm5      = 0;
error.int_arm6      = 0;
error.last_arm1     = 0;
error.last_arm2     = 0;
error.last_arm3     = 0;
error.last_arm4     = 0;
error.last_arm5     = 0;
error.last_arm6     = 0;
ini_armM1   = [0;0;0];%施加在关节的力矩在局部坐标系下的坐标
ini_armM2   = [0;0;0];
ini_armM3   = [0;0;0];
ini_armM4   = [0;0;0];
ini_armM5   = [0;0;0];
ini_armM6   = [0;0;0];
u0          = [ini_armM1;ini_armM2;ini_armM3;ini_armM4;ini_armM5;ini_armM6];%18*1
ulog        = repmat(u0,1,N+1);

des_pos                 = zeros(6,N);
des_pos(1,:)            = pi/6;
des_pos(2,:)            = -pi/2;
% initial calculate
Mi      = blkdiag(robotarm.m1,robotarm.j1,robotarm.m2,robotarm.j2,robotarm.m3,robotarm.j3, ...
                    robotarm.m4,robotarm.j4,robotarm.m5,robotarm.j5,robotarm.m6,robotarm.j6);
robotarm.Cqa        = zeros(5*robotarm.n,6*robotarm.n);
robotarm.dCqa       = zeros(5*robotarm.n,6*robotarm.n);
robotarm.equ_left   = [Mi,robotarm.Cqa';robotarm.Cqa,zeros(5*robotarm.n,5*robotarm.n)];
robotarm.equ_right  = zeros(11*robotarm.n,1);
robotarm.I1         = [1;0;0];
robotarm.I2         = [0;1;0];
robotarm.I3         = [0;0;1];
% initial plot setting
initial_plot_set_guyueju;
figure
for i = 1:N
    t               = i*dt;
    tlog(1,i+1)     = t;
    dM  = control_parameter_guyueju(State,t,dt,i);
    u   = ulog(:,i)+dM;
    
    State(:,i+1)    = runge_kutta4(@vdn_mechanical_arm,State(:,i),u,t,dt,i);
    plot_realtime_guyueju(State(:,i+1),t);
end
end
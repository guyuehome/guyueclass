%直接在启动时执行，作为外部文件，以变量储存。
%% theta d a alpha revolute（0）
L1=Link([0 0.445 0.150 -pi/2 0]);
L2=Link ([-pi/2 0 0.700 0 0]);
L3=Link([0 0 0.115 -pi/2 0]);
L4=Link([0 0.795 0 pi/2 0]);
L5=Link([0 0 0 -pi/2 0]);
L6=Link([0 0.085 0 0 0]);
L=[L1,L2,L3,L4,L5,L6];
IRB2600 = SerialLink(L,'name','ABB IRB 2600');
%% 
t1=linspace(0,2*pi,100)'; 
t2=linspace(2*pi,0,100)';
t=[t1;t2];
X=sin(t);  %圆柱曲线，椭圆柱曲线
Y=cos(t);
Z=0.1*t+0.2;
plot3(X,Y,Z)
%给定位姿
%% 
P=[X,Y,Z];
Ts=transl(P);
IRB2600.tool=trotx(pi);
q= IRB2600.ikunc(Ts); %ikunc ikine
Jointangle=q;
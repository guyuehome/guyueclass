function q=InvKine(X,Y,Z,inialpos)
%逆运动学求解
%function [q,xyz]=InvKine(X,Y,Z)
L1=Link([0 0.445 0.150 -pi/2 0]);
L2=Link ([-pi/2 0 0.700 0 0]); L2.qlim=[-pi/2 0];
L3=Link([0 0 0.115 -pi/2 0]);  L3.qlim=[-pi/2 pi/2];
L4=Link([0 0.795 0 pi/2 0]);
L5=Link([0 0 0 -pi/2 0]);  L5.qlim=[-pi/2 pi/2];
L6=Link([0 0.085 0 0 0]);
L=[L1,L2,L3,L4,L5,L6];
IRB2600 = SerialLink(L,'name','ABB IRB 2600');
%IRB2600.qlim  %可查看转角限制，为6*2矩阵

%给定位姿
P=[X,Y,Z]
Ts=transl(P);
%IRB2600.tool=trotx(pi);  %用于控制工具末端
%TA=IRB2600.A([1 2 3 4 5 6],inialpos) 
TA=IRB2600.fkine(inialpos); %两种计算方式等价
%逆运动学算法

q= IRB2600.ikunc(Ts*TA);
%q=IRB2600.ikine(TA*Ts);
%q=IRB2600.ikcon(TA*Ts);
end


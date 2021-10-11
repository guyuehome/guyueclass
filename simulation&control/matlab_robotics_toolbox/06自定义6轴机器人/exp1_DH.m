%% D-H参数模型
%Link参数依次为：theta d a alpha
% 对于旋转轴而言，theta参数保持缺省

L1=Link([0 0.445 0.150 -pi/2 0]);
L2=Link ([-90 0 0.900 0 0]); 
L3=Link([0 0 0.150 -pi/2 0]);  
L4=Link([0 0.938 0 pi/2 0]);
L5=Link([0 0 0 -pi/2 0]); 
L6=Link([pi 0.200 0 0 0]);
L=[L1,L2,L3,L4,L5,L6];
L1.qlim=[-180,180]/180*pi;
L2.qlim=[-95 155]/180*pi;
L3.qlim=[-180 75]/180*pi;
L4.qlim=[-175 175]/180*pi;
L5.qlim=[-120 120]/180*pi;
L6.qlim=[-400 400]/180*pi;
IRB2600 = SerialLink(L,'name','ABB IRB 2600');
q0=[0 -90 0 0 0 180]/180*3.14;
view(32,31)
IRB2600.teach(q0)

function plotrealtime(state,i,state_all,t)
global car
% -------------------------------------------------------------------------
% 根据输入的状态变量进行初始计算
qi          = state(1:12,1);
% 计算每个部件的欧拉角
I1          = [1;0;0];
I3          = [0;0;1];
eu1         = state(4:6,1);
eu2         = eu1+I1*state(7,1);
eu3         = eu2+I3*state(8,1);
eu4         = eu1+I3*state(9,1);
eu5         = eu1+I1*state(10,1);
eu6         = eu5+I3*state(11,1);
eu7         = eu1+I3*state(12,1);
% -------------------------------------------------------------------------
% 旋转矩阵
A1      = [cos(eu1(1,1)),-sin(eu1(1,1)),0;sin(eu1(1,1)),cos(eu1(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu1(2,1)),-sin(eu1(2,1));0,sin(eu1(2,1)),cos(eu1(2,1))]*...
          [cos(eu1(3,1)),0,sin(eu1(3,1));0,1,0;-sin(eu1(3,1)),0,cos(eu1(3,1))];
A2      = [cos(eu2(1,1)),-sin(eu2(1,1)),0;sin(eu2(1,1)),cos(eu2(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu2(2,1)),-sin(eu2(2,1));0,sin(eu2(2,1)),cos(eu2(2,1))]*...
          [cos(eu2(3,1)),0,sin(eu2(3,1));0,1,0;-sin(eu2(3,1)),0,cos(eu2(3,1))];
A3      = [cos(eu3(1,1)),-sin(eu3(1,1)),0;sin(eu3(1,1)),cos(eu3(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu3(2,1)),-sin(eu3(2,1));0,sin(eu3(2,1)),cos(eu3(2,1))]*...
          [cos(eu3(3,1)),0,sin(eu3(3,1));0,1,0;-sin(eu3(3,1)),0,cos(eu3(3,1))];
A4      = [cos(eu4(1,1)),-sin(eu4(1,1)),0;sin(eu4(1,1)),cos(eu4(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu4(2,1)),-sin(eu4(2,1));0,sin(eu4(2,1)),cos(eu4(2,1))]*...
          [cos(eu4(3,1)),0,sin(eu4(3,1));0,1,0;-sin(eu4(3,1)),0,cos(eu4(3,1))];
A5      = [cos(eu5(1,1)),-sin(eu5(1,1)),0;sin(eu5(1,1)),cos(eu5(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu5(2,1)),-sin(eu5(2,1));0,sin(eu5(2,1)),cos(eu5(2,1))]*...
          [cos(eu5(3,1)),0,sin(eu5(3,1));0,1,0;-sin(eu5(3,1)),0,cos(eu5(3,1))];
A6      = [cos(eu6(1,1)),-sin(eu6(1,1)),0;sin(eu6(1,1)),cos(eu6(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu6(2,1)),-sin(eu6(2,1));0,sin(eu6(2,1)),cos(eu6(2,1))]*...
          [cos(eu6(3,1)),0,sin(eu6(3,1));0,1,0;-sin(eu6(3,1)),0,cos(eu6(3,1))];
A7      = [cos(eu7(1,1)),-sin(eu7(1,1)),0;sin(eu7(1,1)),cos(eu7(1,1)),0;0,0,1]*...
          [1,0,0;0,cos(eu7(2,1)),-sin(eu7(2,1));0,sin(eu7(2,1)),cos(eu7(2,1))]*...
          [cos(eu7(3,1)),0,sin(eu7(3,1));0,1,0;-sin(eu7(3,1)),0,cos(eu7(3,1))];
R1      = qi(1:3,1);
R2      = R1+A1*car.u12-A2*car.u21;
R3      = R2+A2*car.u23;
R4      = R1+A1*car.u14;
R5      = R1+A1*car.u15-A5*car.u51;
R6      = R5+A5*car.u56;
R7      = R1+A1*car.u17;

Rutire3     = R3+A3*car.u_tire;
Rutire4     = R4+A4*car.u_tire;
Rutire6     = R6+A6*car.u_tire;
Rutire7     = R7+A7*car.u_tire;

R_rod4      = R1+A1*[-0.8;-0.5;-0.1];
R_rod7      = R1+A1*[-0.8;0.5;-0.1];

plot3([Rutire3(1,:),Rutire6(1,:);Rutire4(1,:),Rutire7(1,:)]',[Rutire3(2,:),Rutire6(2,:);Rutire4(2,:),Rutire7(2,:)]',[Rutire3(3,:),Rutire6(3,:);Rutire4(3,:),Rutire7(3,:)]','Color','r','LineWidth',2)
hold on
plot3(state_all(1,1:i),state_all(2,1:i),state_all(3,1:i),'Color','m','LineWidth',2);
hold off
patch('Faces',car.F_body,'Vertices',(R1+A1*(car.V_body'))','FaceColor','g','FaceAlpha',.5);
patch('Faces',car.F_rod,'Vertices',(R2+A2*(car.Vr_rod'))','FaceColor','b','FaceAlpha',.5);
patch('Faces',car.F_rod,'Vertices',(R5+A5*(car.Vl_rod'))','FaceColor','b','FaceAlpha',.5);
patch('Faces',car.F_rod,'Vertices',(R_rod4+A1*(car.Vr_rod'))','FaceColor','b','FaceAlpha',.5);
patch('Faces',car.F_rod,'Vertices',(R_rod7+A1*(car.Vl_rod'))','FaceColor','b','FaceAlpha',.5);
patch('Faces',[1,2,3,4],'Vertices',[-20,-20,0;-20,20,0;20,20,0;20,-20,0],'FaceColor','k','FaceAlpha',.25);

title(sprintf('$$t: %0.3f$$',t),'Interpreter','latex');
axis equal
view(3)
xlim([R1(1,1)-1.5,R1(1,1)+1.5])
ylim([R1(2,1)-1.5,R1(2,1)+1.5])
zlim([-1,2])
pause(0.0002)
end
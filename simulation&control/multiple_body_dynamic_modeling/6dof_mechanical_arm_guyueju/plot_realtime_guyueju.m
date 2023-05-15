function plot_realtime_guyueju(state,t)
global robotarm
po_arm1     = state(1:6,1);%第一个杆的位置（xyz坐标，欧拉角）
po_arm2     = state(13:18,1);
po_arm3     = state(25:30,1);
po_arm4     = state(37:42,1);
po_arm5     = state(49:54,1);
po_arm6     = state(61:66,1);
% -------------------------------------------------------------------------
% 旋转矩阵
A_arm1  = [cos(po_arm1(4,1)),0,sin(po_arm1(4,1));0,1,0;-sin(po_arm1(4,1)),0,cos(po_arm1(4,1))]*...
    [cos(po_arm1(5,1)),-sin(po_arm1(5,1)),0;sin(po_arm1(5,1)),cos(po_arm1(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm1(6,1)),-sin(po_arm1(6,1));0,sin(po_arm1(6,1)),cos(po_arm1(6,1))];
A_arm2  = [cos(po_arm2(4,1)),0,sin(po_arm2(4,1));0,1,0;-sin(po_arm2(4,1)),0,cos(po_arm2(4,1))]*...
    [cos(po_arm2(5,1)),-sin(po_arm2(5,1)),0;sin(po_arm2(5,1)),cos(po_arm2(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm2(6,1)),-sin(po_arm2(6,1));0,sin(po_arm2(6,1)),cos(po_arm2(6,1))];
A_arm3  = [cos(po_arm3(4,1)),0,sin(po_arm3(4,1));0,1,0;-sin(po_arm3(4,1)),0,cos(po_arm3(4,1))]*...
    [cos(po_arm3(5,1)),-sin(po_arm3(5,1)),0;sin(po_arm3(5,1)),cos(po_arm3(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm3(6,1)),-sin(po_arm3(6,1));0,sin(po_arm3(6,1)),cos(po_arm3(6,1))];
A_arm4  = [cos(po_arm4(4,1)),0,sin(po_arm4(4,1));0,1,0;-sin(po_arm4(4,1)),0,cos(po_arm4(4,1))]*...
    [cos(po_arm4(5,1)),-sin(po_arm4(5,1)),0;sin(po_arm4(5,1)),cos(po_arm4(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm4(6,1)),-sin(po_arm4(6,1));0,sin(po_arm4(6,1)),cos(po_arm4(6,1))];
A_arm5  = [cos(po_arm5(4,1)),0,sin(po_arm5(4,1));0,1,0;-sin(po_arm5(4,1)),0,cos(po_arm5(4,1))]*...
    [cos(po_arm5(5,1)),-sin(po_arm5(5,1)),0;sin(po_arm5(5,1)),cos(po_arm5(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm5(6,1)),-sin(po_arm5(6,1));0,sin(po_arm5(6,1)),cos(po_arm5(6,1))];
A_arm6  = [cos(po_arm6(4,1)),0,sin(po_arm6(4,1));0,1,0;-sin(po_arm6(4,1)),0,cos(po_arm6(4,1))]*...
    [cos(po_arm6(5,1)),-sin(po_arm6(5,1)),0;sin(po_arm6(5,1)),cos(po_arm6(5,1)),0;0,0,1]*...
    [1,0,0;0,cos(po_arm6(6,1)),-sin(po_arm6(6,1));0,sin(po_arm6(6,1)),cos(po_arm6(6,1))];
% -------------------------------------------------------------------------
% 绘图
clf
robotarm.gh          = patch('Faces',[1,2,3,4],'Vertices',[-20,-20,0;-20,20,0;20,20,0;20,-20,0],'FaceColor','k','FaceAlpha',.25);
patch('Faces',robotarm.F,'Vertices',(po_arm1(1:3)+A_arm1*(robotarm.V0_arm1)')','FaceColor','g','FaceAlpha',.5);
patch('Faces',robotarm.F,'Vertices',(po_arm2(1:3)+A_arm2*(robotarm.V0_arm2)')','FaceColor','g','FaceAlpha',.5);
patch('Faces',robotarm.F,'Vertices',(po_arm3(1:3)+A_arm3*(robotarm.V0_arm3)')','FaceColor','b','FaceAlpha',.5);
patch('Faces',robotarm.F,'Vertices',(po_arm4(1:3)+A_arm4*(robotarm.V0_arm4)')','FaceColor','m','FaceAlpha',.5);
patch('Faces',robotarm.F,'Vertices',(po_arm5(1:3)+A_arm5*(robotarm.V0_arm5)')','FaceColor','g','FaceAlpha',.5);
patch('Faces',robotarm.F,'Vertices',(po_arm6(1:3)+A_arm6*(robotarm.V0_arm6)')','FaceColor','r','FaceAlpha',.5);

box on ;%边框
grid on;%网格
xlabel('$$X(m)$$','Interpreter','latex','FontSize',15);
ylabel('$$Y(m)$$','Interpreter','latex','FontSize',15);
zlabel('$$Z(m)$$','Interpreter','latex','FontSize',15);
axis equal
xlim([-0.5,2])
ylim([-1,1.5])
zlim([-0.5,2])
view(45,25)

title(sprintf('动画t:%0.3f',t));
pause(0.0002)
end
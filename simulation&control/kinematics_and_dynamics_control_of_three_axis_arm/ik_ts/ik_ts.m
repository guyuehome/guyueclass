clear; clc; close all; format compact;
% 标准的DH参数
alpha = [90, 0, 0];%角度
a = [ 0, 0.210, 0.180];%米
d = [0.0, 0.0, 0.0];%米
offset = [0, 0, 0];%米
DH = [alpha', a', d', offset'];
qlim = deg2rad([-360, 360; -360, 360;-360, 360;]);%角度

% 建立机械臂模型
for i = 1:length(alpha)
    L(i) = Revolute('d', d(i),'a', a(i), 'alpha',deg2rad(alpha(i)),...
        'offset', deg2rad(offset(i)),'qlim', qlim(i,:));
end
robot = SerialLink(L,'name', 'Qing-arm');
base = transl(0,0,0)*rpy2tr(0,0,0,'deg');
robot.base = base;
tool = transl(0,0,0)*rpy2tr(0,0,0,'deg','xyz');
robot.tool = tool;
robot.teach;
robot.display();
hold on;















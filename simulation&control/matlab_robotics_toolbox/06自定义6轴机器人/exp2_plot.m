%% 准备模型和绘图点
%建立D-H模型
%exp1_DH
% 导入数学曲线的点
%exp2_points
%% 机器人反解
trail=[];
for i=1:length(points)
   T1 = transl(points(i,:))*trotx(pi) ; 
  %q1= IRB2600.ikunc(T1);
  %q1=IRB2600.ikine(T1);
  q1=IRB2600.ikcon(T1);
   trail=[trail;q1];
end

IRB2600.plot(trail,'trail','k')
figure(2)
plot(trail)
title("关节角变化曲线")
figure(3)
plot3(px,py,pz)
title("末端轨迹曲线")


%当程序执行完毕，执行此文件，将所有的点连成线
u=simout;
X=u(:,1); %取第一列数据
Y=u(:,2);
Z=u(:,3);
% t=[0:pi/10:5*pi]'; 
% X=2*sin(t)+0.5;  %圆柱曲线，椭圆柱曲线
% Y=cos(t)+0.5;
% Z=0.1*t;
plot3(X,Y,Z);
xlabel('x轴')
ylabel('y轴')
zlabel('z轴')
title('末端轨迹——鼠标响应')



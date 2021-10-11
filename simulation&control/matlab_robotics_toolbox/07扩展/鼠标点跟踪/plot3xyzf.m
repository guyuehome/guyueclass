function  plot3xyzf(u)
%PLOT3XYZF 此处显示有关此函数的摘要
X=u(:,1); %取第一列数据
Y=u(:,2);
Z=u(:,3);
plot3(X,Y,Z,'o');
xlabel('x轴')
ylabel('y轴')
zlabel('z轴')
title('末端轨迹——熊')


end


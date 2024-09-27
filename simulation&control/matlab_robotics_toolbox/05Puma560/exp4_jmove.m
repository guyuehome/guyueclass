%齐次位姿：
T1 = transl(0.4,0.2,0)*trotx(pi) ;
T2 = transl(0.4,-0.2,0)*trotx(pi/2);
%关节量：
q1=p560.ikine6s(T1); q2=p560.ikine6s(T2);
%运动时间
t=[0:0.05:2]';

%标量插补，多轴驱动  
q=mtraj(@tpoly,q1,q2,t);
figure(1)
plot(q)

%其结果是得到一个50×6阶的矩阵a，其中每行对应一个采样时间步，每列对应一个关节值。
%绘制轨迹  
figure(2)
p560.plot(q,'trail','k')

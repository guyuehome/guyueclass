%齐次位姿：
T1 = transl(0.4,0.2,0)*trotx(pi) ;
T2 = transl(0.4,-0.2,0)*trotx(pi/2);

%对于许多应用，我们都需要在笛卡儿空间中的直线运动，这称为笛卡儿运动。
t=[0:0.05:2]';
Ts=ctraj(T1,T2,length(t));
Ts=trinterp(T1,T2,[0:0.05:2]); 
%其输入参数是初始和最后的位姿，以及时间步数，它返回的是一个三维矩阵形式的轨迹。

figure(1)
%位置分量：
%plot(t,transl(Ts));
%姿态分量：
%plot(t,tr2rpy(Ts))

figure(2)
%通过逆运动学求解，我们可以得到相应的关节空间轨迹：
qc = p560.ikine6s(Ts);
figure(2)
p560.plot(qc,'trail','k')




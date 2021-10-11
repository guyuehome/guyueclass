%% 姿态和位置 插值
% 其中平移部分使用线性插值，旋转部分使用四元数插值法 interp进行球形插值。 
T0=transl(0.4,0.2,0)*trotx(pi); 
T1=transl(-0.4,-0.2,0.3)*troty(pi/2)*trotz(-pi/2); 

%两个位姿之间50个分步之间的轨迹用以下方法生成 
Ts=trinterp(T0,T1,[0:49]/49); 
%动画 
%tranimate(Ts)
Ts_pos=transl(Ts);
figure(1)
plot(Ts_pos)
Ts_roi=t2r(Ts);
figure(2)
tranimate(Ts_roi)

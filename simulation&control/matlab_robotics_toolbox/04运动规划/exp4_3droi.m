%% 姿态旋转插值

%例如：定义两个姿态。 
R0=rotz(-1)*roty(-1); R1=rotz(1)*roty(1);
%得到等价横滚-俯仰-偏航角 
rpy0=tr2rpy(R0) ; 
rpy1=tr2rpy(R1); 
%分50个时间步在它们之间生成一条轨迹： 
rpy=mtraj(@tpoly,rpy0,rpy1,50);
%动画演示 
tranimate(rpy2tr(rpy));
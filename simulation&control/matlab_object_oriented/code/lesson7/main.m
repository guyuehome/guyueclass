clear;
%优化范围
xmin=[-5,-5,-5];
xmax=[5,5,5];
Gm=100;%最大代数
Np=50;%种群个体数
M=2;%目标变量数

obj=nsga2(xmin,xmax,Gm,Np,M,@calModel);
obj.saveG=10;
obj.saveSample = true;

obj.initialpopulation;
obj.train;

obj.plotCosts('ParetoOptimalFront');
obj.writeLog;
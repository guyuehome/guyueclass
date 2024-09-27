% 第4讲：遗传算法
% 作者： Ally
% 日期： 2021/07/25
clc
clear
close all

%% 三维路径规划模型
startPos = [1, 1, 1];
goalPos = [100, 100, 80];

% 定义山峰地图
posBound = [0,100; 0,100; 0,100;];

% 地图长、宽、高范围
[X,Y,Z] = defMap(posBound);

%% 设置超参数
chromLength = 5;     % 染色体长度，代表路线的控制点数，未加首末两点
p_select = 0.5;      % 选择概率
p_crs = 0.8;         % 交叉概率
p_mut = 0.2;         % 变异概率
popNum = 50;         % 种群规模
iterMax = 100;       % 最大迭代数

%% 种群初始化
% 产生初始种群   
pop = initPop(popNum,chromLength,posBound);

% 计算种群适应度
pop = calFitness(startPos, goalPos, X,Y,Z,pop);

% 更新种群最优
GlobalBest.fitness = inf; % 初始化每一代的最优粒子
[pop,GlobalBest] = calBest(pop,GlobalBest); 

%% 主程序
for i = 1:iterMax    
    % 选择操作
    parentPop = select(pop, p_select);

    % 交叉操作
    childPop = crossover(parentPop,p_crs);
    
    % 变异操作
    childPop = mutation(childPop,p_mut,posBound);
    
    % 将父代和子代组合得到新的种群
    pop = [parentPop, childPop];
    
    % 计算种群适应度
    pop = calFitness(startPos, goalPos, X,Y,Z,pop);

    % 更新种群最优
    [pop,GlobalBest] = calBest(pop,GlobalBest);
    
    % 把每一代的最优粒子赋值给fitness_beat_iters
    fitness_beat_iters(i) = GlobalBest.fitness;
    
    % 在命令行窗口显示每一代的信息
    disp(['第' num2str(i) '代:' '最优适应度 = ' num2str(fitness_beat_iters(i))]);
    
    % 画图
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest);
    pause(0.001);
end

% 理论最小适应度：直线距离
fitness_best = norm(startPos - goalPos);
disp([ '理论最优适应度 = ' num2str(fitness_best)]);

% 画适应度迭代图
figure
plot(fitness_beat_iters,'LineWidth',2);
xlabel('迭代次数');
ylabel('最优适应度');

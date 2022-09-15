clc
clear
close all

%% 创建道路环境
numCars = 30;                % 每条道路车流量
numLanes = 3;                % 车道数量
laneWidth = 3.5;             % 车道宽度
laneLength = 200;            % 车道长度
solidLineLength = 30;        % 车道实线长度
dt = 0.1;                    % 步长时间
averageDesiredSpeed = 20;    % 非控制路段的目标平均车速,m/s
sigma = 3;                   % 定义初始速度的偏差系数

%% 初始化，构造场景对象
[par,Scenario,vehInfo,intersectionGoalPos,TL_ini] = initiateScenario...
    (numCars,numLanes,laneWidth,laneLength,dt,averageDesiredSpeed,sigma);

%% 画场景图以及红绿灯
plot(Scenario)
hold on
ax = gca;
plotTL(TL_ini,ax)

%% 步进仿真
exitID = [];
steps = Scenario.StopTime / Scenario.SampleTime;
for  step = 1:steps
    % 调用函数步进场景
    [Scenario,vehInfo,exitID] = stepScenario(par,Scenario,vehInfo,...
        intersectionGoalPos,exitID);
    
    % 步进场景动画
    advance(Scenario);
    
    % 更新信号灯状态
    TL = updateTL(TL_ini,Scenario.SimulationTime,ax);
    
    % 创建GIF动画
    MakeGif('demo.gif',step)
end

    
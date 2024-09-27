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

%% 构造驾驶场景对象
Scenario = drivingScenario;        % 初始化自动驾驶场景
Scenario.SampleTime = dt;          % 步长时间
Scenario.StopTime = 120;           % 仿真结束时间
par.numLanes = numLanes;
par.numCars = numCars;
par.laneWidth = laneWidth;
par.laneLength = laneLength;
par.laneCenters = laneWidth*([0:numLanes-1]) + laneWidth/2;

% 竖直和水平两个方向道路
roadCenters_1 = [laneLength 0 0; -laneLength 0 0];
roadCenters_2 = [0 laneLength 0; 0 -laneLength 0];

% 每条道路设置分界线标识
marking = laneMarking('Solid');
for k = 1:numLanes-1
    marking = [marking laneMarking('Dashed')];
end
marking = [marking  laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])];
for k = 1:numLanes-1
    marking = [marking laneMarking('Dashed')];
end
marking = [marking laneMarking('Solid')];
laneSpecification = lanespec(...
    numLanes*2,...
    'Width',laneWidth,...
    'Marking', marking);

% 生成道路
road(Scenario, roadCenters_1, 'Lanes', laneSpecification);
road(Scenario, roadCenters_2, 'Lanes', laneSpecification);

%%
traffic_light = struct;
% 1号入口交通灯位置
traffic_light(1).left.pos = [par.laneWidth/2, -par.laneWidth*par.numLanes, 0];
traffic_light(1).left.state = 'red';
traffic_light(1).stright.pos = [3*par.laneWidth/2, -par.laneWidth*par.numLanes, 0];
traffic_light(1).stright.state = 'red';
traffic_light(1).right.pos = [5*par.laneWidth/2, -par.laneWidth*par.numLanes, 0];
traffic_light(1).right.state = 'green';

% 2号进口交通灯位置
traffic_light(2).left.pos = [par.laneWidth*par.numLanes, par.laneWidth/2, 0];
traffic_light(2).left.state = 'red';
traffic_light(2).stright.pos = [par.laneWidth*par.numLanes, 3*par.laneWidth/2, 0];
traffic_light(2).stright.state = 'red';
traffic_light(2).right.pos = [par.laneWidth*par.numLanes, 5*par.laneWidth/2, 0];
traffic_light(2).right.state = 'green';

% 3号进口交通灯位置
traffic_light(3).left.pos = [-par.laneWidth/2, par.laneWidth*par.numLanes, 0];
traffic_light(3).left.state = 'red';
traffic_light(3).stright.pos = [-3*par.laneWidth/2, par.laneWidth*par.numLanes, 0];
traffic_light(3).stright.state = 'red';
traffic_light(3).right.pos = [-5*par.laneWidth/2, par.laneWidth*par.numLanes, 0];
traffic_light(3).right.state = 'green';

% 4号进口交通灯位置
traffic_light(4).left.pos = [-par.laneWidth*par.numLanes, -par.laneWidth/2, 0];
traffic_light(4).left.state = 'red';
traffic_light(4).stright.pos = [-par.laneWidth*par.numLanes, -3*par.laneWidth/2, 0];
traffic_light(4).stright.state = 'red';
traffic_light(4).right.pos = [-par.laneWidth*par.numLanes, -5*par.laneWidth/2, 0];
traffic_light(4).right.state = 'green';


%% 画图
plot(Scenario)
xlim([-40,40]);
ylim([-40,40]);
hold on
for t = 0:120
    % 更新红绿灯状态
    traffic_light = updateTL(traffic_light,t);
    if ismember(t,[1 30 33 58 60 90 93 118 120])
        % 画红绿灯
        for i=1:length(traffic_light)
            % 左转灯
            plot(traffic_light(i).left.pos(1), traffic_light(i).left.pos(2), 'Marker','o',...
                'MarkerFaceColor',traffic_light(i).left.state,'MarkerSize',8);
            
            % 直行灯
            plot(traffic_light(i).stright.pos(1), traffic_light(i).stright.pos(2), 'Marker','o',...
                'MarkerFaceColor',traffic_light(i).stright.state,'MarkerSize',8);
            
            % 右转灯
            plot(traffic_light(i).right.pos(1), traffic_light(i).right.pos(2), 'Marker','o',...
                'MarkerFaceColor',traffic_light(i).right.state,'MarkerSize',8);
        end
        pause(2)
    end
    
end

%% 函数
function traffic_light_update = updateTL(traffic_light,t)
% 四相位信号交叉口，周期长度为120s
traffic_light_update = traffic_light;
t = mod(t,120);
t = round(t,1);

% 相位1：东西直行，绿30s，黄3s
if t >= 0 && t <30
    traffic_light_update(1).stright.state = 'green';
    traffic_light_update(3).stright.state = 'green';
elseif t >= 30 && t < 33
    traffic_light_update(1).stright.state = 'yellow';
    traffic_light_update(3).stright.state = 'yellow';
end

% 相位2：东西左转，绿25s，黄2s
if t >= 33 && t < 58
    traffic_light_update(1).stright.state = 'red';
    traffic_light_update(3).stright.state = 'red';
    traffic_light_update(1).left.state = 'green';
    traffic_light_update(3).left.state = 'green';
elseif t >= 58 && t < 60
    traffic_light_update(1).left.state = 'yellow';
    traffic_light_update(3).left.state = 'yellow';
end

% 相位3：南北直行，绿30s，黄3s
if t >= 60 && t < 90
    traffic_light_update(1).left.state = 'red';
    traffic_light_update(3).left.state = 'red';
    traffic_light_update(2).stright.state = 'green';
    traffic_light_update(4).stright.state = 'green';
elseif t >= 90 && t < 93
    traffic_light_update(2).stright.state = 'yellow';
    traffic_light_update(4).stright.state = 'yellow';
end

% 相位4：南北左转，绿25s，黄3s
if t >= 93 && t < 118
    traffic_light_update(2).stright.state = 'red';
    traffic_light_update(4).stright.state = 'red';
    traffic_light_update(2).left.state = 'green';
    traffic_light_update(4).left.state = 'green';
elseif t >= 118 && t < 120
    traffic_light_update(2).left.state = 'yellow';
    traffic_light_update(4).left.state = 'yellow';
end
end
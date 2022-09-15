clc
clear
close all

%% 创建道路环境
numCars = 30;                % 每条道路车流量
numLanes = 3;                % 车道数量
laneWidth = 3.5;             % 车道宽度
laneLength = 200;            % 车道长度
dt = 0.1;                    % 步长时间

%% 构造驾驶场景对象
Scenario = drivingScenario;        % 初始化自动驾驶场景
Scenario.SampleTime = dt;          % 步长时间
Scenario.StopTime = 120;           % 仿真结束时间
par.numLanes = numLanes;
par.numCars = numCars;
par.laneWidth = laneWidth;
par.laneLength = laneLength;  
par.laneCenters = laneWidth*(0:numLanes-1) + laneWidth/2;

% 竖直和水平两个方向道路
roadCenters_1 = [laneLength 0 0; -laneLength 0 0];
roadCenters_2 = [0 laneLength 0; 0 -laneLength 0];
  
% 每条道路设置分界线标识
marking = laneMarking('Solid');   % 实线
for k = 1:numLanes-1
    marking = [marking laneMarking('Dashed')];   % 2条虚线
end
marking = [marking  laneMarking('DoubleSolid', 'Color', [0.98 0.86 0.36])];% 双实线
for k = 1:numLanes-1
    marking = [marking laneMarking('Dashed')]; % 2条虚线
end
marking = [marking laneMarking('Solid')];  % 实线
laneSpecification = lanespec(...
    numLanes*2,...
    'Width',laneWidth,...
    'Marking', marking);

% 生成道路
road(Scenario, roadCenters_1, 'Lanes', laneSpecification);
road(Scenario, roadCenters_2, 'Lanes', laneSpecification);

%% 交通灯
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
% 画场景图
plot(Scenario)
xlim([-20,20]);
ylim([-20,20]);
hold on

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

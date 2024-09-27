function [par,Scenario,vehInfo,intersectionGoalPos,TL_ini] = initiateScenario...
    (numCars,numLanes,laneWidth,laneLength,dt,AverageDesiredSpeed,Sigma)
%% 初始化场景，包括建立十字路口道路、信号灯、输入初始车流、构造Stateflow驾驶员等
Scenario = drivingScenario;        % 初始化自动驾驶场景
Scenario.SampleTime = dt;          % 步长时间
Scenario.StopTime = 120;           % 仿真结束时间
par.numLanes = numLanes;
par.numCars = numCars;
par.laneWidth = laneWidth;
par.laneLength = laneLength;  
par.laneCenters = laneWidth*([0:numLanes-1]) + laneWidth/2;

% 根据stateFlow定义驾驶员结构体
vehInfo = struct;
vehInfo.ID = [];                   % 车辆在整个场景中的编号 
vehInfo.Drivers = [];              % 赋值驾驶员    
% vehInfo.pos = [];                  % 当前车辆的位置
% vehInfo.v = [];                    % 当前车辆的速度
vehInfo.entryIdx = [];             % 车辆进入路段的编号
vehInfo.linkIdx = [];              % 车辆当前位于路段的编号
vehInfo.laneIdx = [];              % 车辆当前位于车道的编号
vehInfo.passFlag = 0;              % 是否经过交叉口的标识符
vehInfo.turnPath = 0;              % 车辆经过交叉口的参考路径
vehInfo = repmat(vehInfo,4*numCars,1);

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

%% 右、上、左、下四个进口方向输入车辆
% 注意这里的1,2,3,4是指进入场景的4条路段，与编号不同
for i = 1:4
    for j = 1:numCars
        % 随机定义车辆的起始纵横垂位置
        latPos = par.laneCenters(randi([1 size(par.laneCenters,2)],1));
        longPos = rand;
        pos_LLV = [longPos latPos 0];
        
        % 定义纵横垂随机速度
        speed = AverageDesiredSpeed + Sigma*randn(1);
        Vel_LLV = [speed 0 0];
        
        % 将位置和速度的LLV坐标转为XYZ坐标
        [pos_XYZ,Vel_XYZ] = transferCoord(pos_LLV,Vel_LLV,2,i*2-1);
    
        % 在场景添加车辆
        yawSet = [90, 180, -90,0]; % 四个方向的初始航向角
        actor = vehicle(Scenario, ...
            'ClassID', 1, ...
            'Position', pos_XYZ,...
            'Yaw',yawSet(i),...
            'Velocity', Vel_XYZ);
        
        % 预设一条轨迹，主要用于实现车辆先后进入场景
        pos_LLV_1 = pos_LLV + [1 0 0];
        [pos_XYZ_1,~] = transferCoord(pos_LLV_1,[0 0 0],2,i*2-1);
        waypoints = [pos_XYZ; pos_XYZ_1];
        trajectory(actor, waypoints,[Vel_LLV(1), Vel_LLV(1)]);
         
        % 随机定义车辆出现在道路的时间
        Scenario.Actors(end).EntryTime = j*2;
  
        % 为车辆添加staflow驾驶员
        if i ==1 && j == 1
            animate = true;
        else
            animate = false;
        end
        vehInfo((i-1)*numCars+j).ID = (i-1)*numCars+j;
        vehInfo((i-1)*numCars+j).Drivers = sf_driver(...
            '-enableAnimation',animate,...
            '-animationDelay',1,...
            'laneCenters',par.laneCenters,...
            'laneWidth',par.laneWidth,...
            'maxSpeed',speed,...
            'myPos',pos_LLV,...
            'numCars',numCars,...
            'me',j,...
            'isZoneOccupied',0);
        
        % 车辆的车道编号
        vehInfo((i-1)*numCars+j).linkIdx = i*2-1;
        vehInfo((i-1)*numCars+j).entryIdx = i*2-1;
    end
end

%% 交叉路口的目标点
intersectionGoalPos = struct;
intersectionGoalPos.turnLeft = [];
intersectionGoalPos.turnStright = [];
intersectionGoalPos.turnRight = [];

% 1号入口
intersectionGoalPos(1).turnLeft = [-par.laneWidth*par.numLanes-0.1, par.laneWidth/2,0];
intersectionGoalPos(1).turnStright = [3*par.laneWidth/2,  par.laneWidth*par.numLanes+0.1,0];
intersectionGoalPos(1).turnRight = [par.laneWidth*par.numLanes+0.1, -5*par.laneWidth/2,0];

% 2号入口
intersectionGoalPos(2).turnLeft  = [-par.laneWidth/2, -par.laneWidth*par.numLanes-0.1,0];
intersectionGoalPos(2).turnStright = [-par.laneWidth*par.numLanes-0.1, 3*par.laneWidth/2,0];
intersectionGoalPos(2).turnRight = [ 5*par.laneWidth/2, par.laneWidth*par.numLanes+0.1,0];

% 3号入口
intersectionGoalPos(3).turnLeft= [par.laneWidth*par.numLanes+0.1, -par.laneWidth/2,0];
intersectionGoalPos(3).turnStright = [-3*par.laneWidth/2  -par.laneWidth*par.numLanes-0.1,0];
intersectionGoalPos(3).turnRight = [ -par.laneWidth*par.numLanes-0.1, 5*par.laneWidth/2, 0];

% 4号入口
intersectionGoalPos(4).turnLeft = [par.laneWidth/2, par.laneWidth*par.numLanes+0.1,0];
intersectionGoalPos(4).turnStright = [par.laneWidth*par.numLanes+0.1, -3*par.laneWidth/2,0];
intersectionGoalPos(4).turnRight = [ -5*par.laneWidth/2, -par.laneWidth*par.numLanes-1,0];

%% 交通灯初始信息
TL_ini = struct;
% 1号入口交通灯位置
TL_ini(1).left.pos = [par.laneWidth/2, -par.laneWidth*par.numLanes, 0];
TL_ini(1).left.state = 'red';
TL_ini(1).stright.pos = [3*par.laneWidth/2, -par.laneWidth*par.numLanes, 0];
TL_ini(1).stright.state = 'red';
TL_ini(1).right.pos = [5*par.laneWidth/2, -par.laneWidth*par.numLanes, 0];
TL_ini(1).right.state = 'green';

% 2号进口交通灯位置
TL_ini(2).left.pos = [par.laneWidth*par.numLanes, par.laneWidth/2, 0];
TL_ini(2).left.state = 'red';
TL_ini(2).stright.pos = [par.laneWidth*par.numLanes, 3*par.laneWidth/2, 0];
TL_ini(2).stright.state = 'red';
TL_ini(2).right.pos = [par.laneWidth*par.numLanes, 5*par.laneWidth/2, 0];
TL_ini(2).right.state = 'green';

% 3号进口交通灯位置
TL_ini(3).left.pos = [-par.laneWidth/2, par.laneWidth*par.numLanes, 0];
TL_ini(3).left.state = 'red';
TL_ini(3).stright.pos = [-3*par.laneWidth/2, par.laneWidth*par.numLanes, 0];
TL_ini(3).stright.state = 'red';
TL_ini(3).right.pos = [-5*par.laneWidth/2, par.laneWidth*par.numLanes, 0];
TL_ini(3).right.state = 'green';

% 4号进口交通灯位置
TL_ini(4).left.pos = [-par.laneWidth*par.numLanes, -par.laneWidth/2, 0];
TL_ini(4).left.state = 'red';
TL_ini(4).stright.pos = [-par.laneWidth*par.numLanes, -3*par.laneWidth/2, 0];
TL_ini(4).stright.state = 'red';
TL_ini(4).right.pos = [-par.laneWidth*par.numLanes, -5*par.laneWidth/2, 0];
TL_ini(4).right.state = 'green';

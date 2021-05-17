% 基于栅格地图的机器人路径规划算法
% D_star算法
clc
clear
close all

%% 定义ColorMap
rows = 10;
cols = 20;
[field,cmap] = defColorMap(rows, cols);

% 起始点和目标点
startPos = 2;
goalPos = rows*cols-2;
field(startPos) = 4;
field(goalPos) = 5;

%% 第一阶段：搜索路径

% 初始化Nodes结构体，包含node，t，k，h，parent
field1 = field;
Nodes = struct;
for i = 1:rows*cols
    Nodes(i).node = i;         % 节点线性索引
    Nodes(i).t = 'new';        % 该点的状态
    Nodes(i).k = inf;          % 该点与目标点的累计最小距离
    Nodes(i).h = inf;          % 该点与目标点当前计算距离
    Nodes(i).parent = nan;     % 后向节点
end

% 初始化目标节点
Nodes(goalPos).t = 'open';       % 该点的状态
Nodes(goalPos).k = 0;            % 该点与目标点的累计最小距离
Nodes(goalPos).h = 0;            % 该点与目标点的距离
Nodes(goalPos).parent = nan;     % 后向节点

% 把goalPos放到openList
openList = [goalPos, Nodes(goalPos).k];

% 循环迭代
while true
    [Nodes, openList, k_old] = process_state(field1, Nodes, openList,goalPos);
    
    % 当判断条件为Nodes(startPos).t == 'closed'，程序退出，此时类似A*算法；
    % 当判断条件为isempty(openList)，程序退出，此时类似Dijstra算法；
    if isempty(openList)
        break
    end
end

% 找到路径
node = startPos;
path1 = node;
while true
    path1(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == goalPos
        break
    end
end

% 找出目标最优路径
field1(path1(2:end-1)) = 6;

% 第一阶段路径画图
image(1.5,1.5,field1);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;

%% 第二阶段：遭遇障碍物，重新搜索路径
% 另外弹出图窗
figure
colormap(cmap);

% 障碍物区域
obsNode = path1(8:11);

% 重置场景、路径
field2 = field;
field2(obsNode) = 3;
node = startPos;   % 机器人当前位置
path2 = node;
flag = 0;
while node ~= goalPos
    
    % 定义field颜色
    if flag 
        field2(path2(flag:end)) = 7;
    else
        field2(path2) = 6;
    end
    field2(startPos) = 4;
    field2(goalPos) = 5;
    
    % 画栅格图
    image(1.5,1.5,field2);
    grid on;
    set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
    set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
    axis image;
    pause(0.3)
    
    % 若父节点parentNode是为障碍，则重新执行process_state函数体
    parentNode = Nodes(node).parent;
    if field2(parentNode) == 2 || field2(parentNode) == 3
        % 首先修改父节点的h值
        Nodes(parentNode).h = inf;
        
        % 若node标识为closed，则修改其h值，并添加到openList
        if isequal(Nodes(node).t, 'closed')
            Nodes(node).h = inf;
            [Nodes,openList] = insert(Nodes,openList,node,Nodes(node).h);
        end
        
        while true
            [Nodes, openList, k_min] = process_state(field2, Nodes, openList,goalPos);
            if k_min >= Nodes(node).h
                break
            end
        end
        
        % 标记当前路径path2的数据长度，用于另一种颜色画图
        flag = length(path2);
    end
    
    node = Nodes(node).parent;
    path2(end+1) = node;
end

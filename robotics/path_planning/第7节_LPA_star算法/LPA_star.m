% 基于栅格地图的机器人路径规划算法
% LPA*算法
clc
clear
close all

%% 定义栅格场景
% 栅格界面大小:行数和列数
rows = 10;
cols = 20;
[field,cmap] = defColorMap(rows, cols);

% 起始点和目标点
startPos = 2;
goalPos = rows*cols-2;
field(startPos) = 4;
field(goalPos) = 5;
field1 = field;

%% 第一次规划路径
% 步骤1：initialize ，初始化节点信息结构体
for i = 1:rows*cols
    Nodes(i).node = i;
    Nodes(i).g = inf;
    Nodes(i).rhs = inf;
    Nodes(i).parent = nan;
end
Nodes(startPos).rhs = 0;   % 令目标点的rhs=0

% 由于起始点的rhs=0,g=inf,两者不相等，故添加到队列U中
U(1,1) = startPos;
U(1,2:3) = calculateKey(Nodes(startPos),goalPos, rows, cols);

% 步骤2：computeShortestPath
[Nodes, U] = computeShortestPath(field1, Nodes, U,rows,cols,startPos,goalPos);

% 从目标点倒推，根据父节点信息找到路径
node = goalPos;
path1 = node;
while true
    path1(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == startPos
        break
    end
end
path1 = path1(end:-1:1);

% 将路径信息反映到field1中
field1(path1(2:end-1)) = 6;

% 画栅格图和路径
image(1.5,1.5,field1);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;

%% 第二次规划路径：规划的路径出现障碍物
obsNodes = path1(8:11);
field2 = field;
field2(obsNodes) = 2;

% 找到以obsNode作为父节点的所有受到影响的子节点
influencedChildNnodes = findInfluencedNnodes(field,Nodes,obsNodes);

% UpdateEdgeCost
for i = 1:length(obsNodes)
    obsNode = obsNodes(i);
    Nodes(obsNode).rhs = inf;
    Nodes(obsNode).g = inf;
end
[Nodes,U] =  UpdateEdgeCost(influencedChildNnodes,Nodes,U,rows,cols,startPos);

% computeShortestPath
[Nodes, U] = computeShortestPath(field2, Nodes, U,rows,cols,startPos,goalPos);

% 从目标点倒推，根据父节点信息找到路径
node = goalPos;
path2 = node;
while true
    path2(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == startPos
        break
    end
end
path2 = path2(end:-1:1);

% 重新画栅格图
figure
colormap(cmap);
field2(obsNodes) = 3;
field2(path2) = 7;
field2(startPos) = 4;
field2(goalPos) = 5; 
image(1.5,1.5,field2);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;

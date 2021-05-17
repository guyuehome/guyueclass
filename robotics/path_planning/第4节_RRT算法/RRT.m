% 基于栅格地图的机器人路径规划算法
% 第4节：RRT算法
clc
clear
close all

%% 障碍物、空白区域、起始点、目标点定义

% 行数和列数
rows = 30;
cols = 50;
[field,cmap] = defColorMap(rows, cols);

% 起点、终点、障碍物区域
startPos = 2;
goalPos = rows*cols-2;
field(startPos) = 4;
field(goalPos) = 5;


%% 算法

% 定义树节点，第一列放节点编号，第二列放该节点的父节点
treeNodes = [startPos, 0];

while true
    % 初始化parentNode和childNode
    parentNode = [];
    childNode = [];
    
    % 在地图空间随机采样撒点
    samplePoint = getSamplePoint(field, treeNodes);
    
    % 依次遍历每一个树节点到采样点的距离，取最小值对应的树节点
    for i = 1:size(treeNodes,1)
        [row_treeNode, col_treeNode] = ind2sub([rows, cols], treeNodes(i,1));
        [row_samplePoint, col_samplePoint] = ind2sub([rows, cols], samplePoint);
        dist(i) = norm([row_treeNode, col_treeNode] - [row_samplePoint, col_samplePoint]);
    end
    [~,idx] = min(dist);
    parentNode = treeNodes(idx,1);
    
    % 生成新的子节点,行列坐标
    childNode = getChildNode(field, parentNode, samplePoint);
    
    % 判断该子节点是否超过地图限制
    if childNode(1) < 1 || childNode(1) > rows ||...
            childNode(2) < 1 || childNode(2) > cols
        continue
    else
        % 转为线性索引
        childNode = sub2ind([rows, cols], childNode(1), childNode(2));
    end

    
    % 判断父节点与子节点的连线是否跨过障碍物
    flag = judgeObs(field, parentNode, childNode);
    if flag
        continue
    end
    
    % 判断该子节点是否已经存在于treeNodes，未在则追加到treeNodes
    if ismember(childNode, treeNodes(:,1))
        continue
     else
        treeNodes(end+1,:) = [childNode, parentNode];
    end
    
    % 判断子节点是否位于目标区域
    [row_childNode, col_childNode] = ind2sub([rows, cols], childNode);
    [row_goalPos, col_goalPos] = ind2sub([rows, cols], goalPos);
    if abs(row_childNode - row_goalPos) + ...
            abs(col_childNode - col_goalPos) < 2
        break
    end
 
end


%% 找出目标最优路径

% 最优路径
path_opt = [];
idx = size(treeNodes,1);
while true
    path_opt(end+1) = treeNodes(idx,1);
    parentNode = treeNodes(idx,2);
    if parentNode == startPos
        break;
    else
        idx = find(treeNodes(:,1) == parentNode);
    end    
end

% 路径信息反映到field中
field(treeNodes(:,1)) = 3;
field(path_opt) = 6;
field(startPos) = 4;
field(goalPos) = 5;

%% 画栅格图 
image(1.5,1.5,field);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;
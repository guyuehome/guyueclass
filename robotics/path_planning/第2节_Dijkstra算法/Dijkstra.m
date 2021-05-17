% 基于栅格地图的机器人路径规划算法
% 第2节：Dijkstra算法
clc
clear
close all

%% 栅格界面、场景定义
% 行数和列数
rows = 10;
cols = 20;
[field,cmap] = defColorMap(rows, cols);

% 起点、终点、障碍物区域
startPos = 2;
goalPos = rows*cols-2;
field(startPos) = 4;
field(goalPos) = 5;

%% 算法初始化
% S/U的第一列表示栅格节点线性索引编号
% 对于S，第二列表示从源节点到本节点已求得的最小距离，不再变更；
% 对于U，第二列表示从源节点到本节点暂时求得的最小距离，可能会变更
U(:,1) = (1: rows*cols)';
U(:,2) = inf;
S = [startPos, 0];
U(startPos,:) = [];

% 更新起点的邻节点及代价
neighborNodes = getNeighborNodes(rows, cols, startPos, field);
for i = 1:8
    childNode = neighborNodes(i,1);
    
    % 判断该子节点是否存在
    if ~isinf(childNode)
        idx = find(U(:,1) == childNode);
        U(idx,2) = neighborNodes(i,2);
    end
end



% S集合的最优路径集合
for i = 1:rows*cols
    path{i,1} = i;
end
for i = 1:8
    childNode =  neighborNodes(i,1);
    if ~isinf(neighborNodes(i,2))
        path{childNode,2} = [startPos,neighborNodes(i,1)];
    end
end


%% 循环遍历
while ~isempty(U)
    
    % 在U集合找出当前最小距离值的节点,视为父节点，并移除该节点至S集合中
    [dist_min, idx] = min(U(:,2));
    parentNode = U(idx, 1);
    S(end+1,:) = [parentNode, dist_min];
    U(idx,:) = [];
    
    % 获得当前节点的临近子节点
    neighborNodes = getNeighborNodes(rows, cols, parentNode, field);

    % 依次遍历邻近子节点，判断是否在U集合中更新邻节点的距离值
    for i = 1:8
        
        % 需要判断的子节点
        childNode = neighborNodes(i,1);
        cost = neighborNodes(i,2);
        if ~isinf(childNode)  && ~ismember(childNode, S)
            
            % 找出U集合中节点childNode的索引值
            idx_U = find(childNode == U(:,1));            
            
            % 判断是否更新
            if dist_min + cost < U(idx_U, 2)
                U(idx_U, 2) = dist_min + cost;
                
                % 更新最优路径
                path{childNode, 2} = [path{parentNode, 2}, childNode];
            end
        end
    end
end


%% 画栅格界面
% 找出目标最优路径
path_opt = path{goalPos,2};
field(path_opt(2:end-1)) = 6;

% 画栅格图
image(1.5,1.5,field);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;
% 基于栅格地图的机器人路径规划算法
% A*算法
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

%% 预处理

% 初始化closeList
parentNode = startPos;
closeList = [startPos,0];

% 初始化openList
openList = struct;
childNodes = getChildNode(field,closeList,parentNode);
for i = 1:length(childNodes)
    [row_startPos,col_startPos] = ind2sub([rows, cols],startPos);
    [row_goalPos,col_goalPos] = ind2sub([rows, cols],goalPos);   
    [row,col] = ind2sub([rows, cols],childNodes(i));

    % 存入openList结构体
    openList(i).node = childNodes(i);
    openList(i).g = norm([row_startPos,col_startPos] - [row,col]);
    openList(i).h = abs(row_goalPos - row) + abs(col_goalPos - col);
    openList(i).f = openList(i).g + openList(i).h;
end

% 初始化path
for i = 1:rows*cols
    path{i,1} = i;
end
for i = 1:length(openList)
    node = openList(i).node;
    path{node,2} = [startPos,node];
end 

%% 开始搜索
% 从openList开始搜索移动代价最小的节点
[~, idx_min] = min([openList.f]);
parentNode = openList(idx_min).node;

% 进入循环
while true  
    
    % 找出父节点的8个子节点，障碍物节点用inf，
    childNodes = getChildNode(field, closeList,parentNode);
    
    % 判断这些子节点是否在openList中，若在，则比较更新；没在则追加到openList中
    for i = 1:length(childNodes)
        
        % 需要判断的子节点
        childNode = childNodes(i);
        [in_flag,idx] = ismember(childNode, [openList.node]);
           
        % 计算代价函数
        [row_parentNode,col_parentNode] = ind2sub([rows, cols], parentNode);
        [row_childNode,col_childNode] = ind2sub([rows, cols], childNode);
        [row_goalPos,col_goalPos] = ind2sub([rows, cols],goalPos);
        g = openList(idx_min).g + norm( [row_parentNode,col_parentNode] -...
            [row_childNode,col_childNode]);
        h = abs(row_goalPos - row_childNode) + abs(col_goalPos - col_childNode);
        f = g + h;
        
        if in_flag   % 若在，比较更新g和f        
            if f < openList(idx).f
                openList(idx).g = g;
                openList(idx).h = h;
                openList(idx).f = f;
                path{childNode,2} = [path{parentNode,2}, childNode];
            end
        else         % 若不在，追加到openList
            openList(end+1).node = childNode;
            openList(end).g = g;
            openList(end).h = h;
            openList(end).f = f;
            path{childNode,2} = [path{parentNode,2}, childNode];
        end
    end
       
    % 从openList移出移动代价最小的节点到closeList
    closeList(end+1,: ) = [openList(idx_min).node, openList(idx_min).f];
    openList(idx_min)= [];
 
    % 重新搜索：从openList搜索移动代价最小的节点
    [~, idx_min] = min([openList.f]);
    parentNode = openList(idx_min).node;
    
    % 判断是否搜索到终点
    if parentNode == goalPos
        closeList(end+1,: ) = [openList(idx_min).node, openList(idx_min).f];
        break
    end
end

%% 画路径
% 找出目标最优路径
path_target = path{goalPos,2};
field(path_target(2:end-1)) = 6;

% 画栅格图
image(1.5,1.5,field);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;
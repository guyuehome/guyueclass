% 基于栅格地图的机器人路径规划算法
% 第8节：D* lite算法
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
Nodes(goalPos).rhs = 0;   % 令目标点的rhs=0

% 第一次路径规划时，km(key修正值)为0
km = 0;

% 由于起始点的rhs=0,g=inf,两者不相等，故添加到队列U中
U(1,1) = goalPos;
U(1,2:3) = calculateKey(Nodes(goalPos),startPos, km, rows, cols);

% 步骤2：computeShortestPath
[Nodes, U] = computeShortestPath(field1, Nodes, U, km,rows,cols,startPos,goalPos);

% 从起始点开始，根据父节点信息找到路径
node = startPos;
path1 = node;
while true
    path1(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == goalPos
        break
    end
end

% 将路径信息反映到field1中
field1(path1(2:end-1)) = 6;

% 画栅格图及路径
image(1.5,1.5,field1);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;

%% 步骤5：机器人沿着路径运动，规划路径前方出现障碍物
% 将field和path赋值
field2 = field;
path2 = path1;

% 定义障碍物节点位置、s_start、s_last
obsRange = 8:11;               % 障碍物位于规划路径的范围
obsNodes = path2(obsRange);    % 障碍物栅格
obsPreviewRange = 2;           % 障碍物预瞄距离
s_start = path2(1);
s_last = s_start;

% 新图窗
figure
colormap(cmap);
flag = 0;        % 用于判断栅格场景是否经历了障碍物更新的标识
step = 1;        % 机器人的运动步数

% 机器人开始移动
while s_start ~= goalPos
    
    % 若机器人还未移动到预设障碍物的预瞄距离那一个栅格
    % 则机器人按照path1正常运动
    % 如果flag=1，表明已经经历了障碍物更新，path随之更新，将按照path2运动
    if step ~= obsRange(1) - obsPreviewRange
        s_start = path2(step);
        
        % 定义field颜色
        field2(obsNodes) = 3;
        field2(path2(1:step)) = 6;
        if flag
            field2(path2(obsRange(1) - obsPreviewRange+1:step)) = 7;
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
       
    % 当机器人移动到预设障碍物的预瞄距离那一个栅格
    % 则进入步骤5，更新所有受到影响的节点代价，并重新寻路
    else 
        
        % 首先在命令窗口弹出语句， 并暂停2秒，模拟重新寻路
        disp('遇到障碍物，重新寻路...')
        pause(2)     
        
        % 更新field2，s_start，km
        field2(obsNodes) = 2;
        s_start = path2(step);
        km = km +  calculateH(s_start, s_last, rows, cols);
        s_last = s_start; 
        
        % 找到以obsNodes作为父节点的所有受到影响的子节点
        influencedChildNnodes = findInfluencedNnodes(field,Nodes,obsNodes);
     
        % 更新所有受到影响的子节点的代价值
        for i = 1:length(obsNodes)
            obsNode = obsNodes(i);
            Nodes(obsNode).rhs = inf;
            Nodes(obsNode).g = inf;
        end
        [Nodes,U] =  UpdateEdgeCost(influencedChildNnodes,Nodes,U,km,rows,cols,s_start);

        % computeShortestPath
        [Nodes, U] = computeShortestPath(field2, Nodes, U, km,rows,cols,s_start,goalPos);
               
        % 从机器人当前位置开始，根据父节点信息找到路径
        node = s_start;
        path2(obsRange(1) - obsPreviewRange + 1:end) = [];
        while true
            path2(end+1) = Nodes(node).parent;
            node = Nodes(node).parent;
            if node == goalPos
                break
            end
        end
             
        flag = 1;
       
    end
    step = step + 1;   
end

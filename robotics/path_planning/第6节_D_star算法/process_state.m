function [Nodes, openList, k_old] = process_state(field, Nodes, openList,goalPos)

% 获得openList中k值最小的节点X及k_old
[k_old,idx] = min(openList(:,2));
X = openList(idx,1);

% 从openList中移出X，并修改其状态为closed
openList(idx,:) = [];
Nodes(X).t = 'closed';

% X的邻节点
neighborNodes = getNeighborNode(field, X);

%% 修正X的父节点及相关信息
% 判断k_old 是否小于 Nodes(X).h,若是表明该节点已经受到障碍的影响
% 那么就判断X的邻节点，看是否能够以某个邻节点作为父节点，使Nodes(X).h变小

if k_old < Nodes(X).h
    
    % 遍历X的邻节点Y
    for i = 1:size(neighborNodes,1)
        Y = neighborNodes(i,1);
        cost_X_Y = neighborNodes(i,2);
        
        % 如果Y点的h值没有上升，并且X的h值能通过Y变得更小,那么就修改X的父节点为Y，重置其h的值。
        % 同时判断Nodes(Y).h <= k_old，考察Y本身是否收到障碍影响导致Nodes(Y).h > k_old
        % 若Nodes(Y).h > k_old，那么再以Y作为父节点就没有意义了
        if Nodes(Y).h <= k_old && Nodes(X).h > Nodes(Y).h + cost_X_Y
            Nodes(X).parent = Y;
            Nodes(X).h = Nodes(Y).h + cost_X_Y;
            % 注意，此处更新了Nodes(X).h，只表明 Nodes(X).h进一步减小了，
            % 但与k_old谁大谁小还尚未可知，需要下一节继续判断
        end
    end
end


%% 上一节修正了X的信息，进一步判断，并修正Y的父节点及相关信息

if k_old == Nodes(X).h   
    % 若k_old = Nodes(X).h，Lower态，有下面三种可能：
    % 1）处于第一遍遍历的阶段；
    % 2）该节点X并没有受到障碍影响；
    % 3）受到了障碍物影响，但是在上一节已经更新了X的parent

    % 遍历X的邻节点Y
    for i = 1:size(neighborNodes,1)
        Y = neighborNodes(i,1);
        cost_X_Y = neighborNodes(i,2);
        if  isequal(Nodes(Y).t, 'new')...
                ||  Nodes(Y).parent == X &&  Nodes(Y).h ~= Nodes(X).h + cost_X_Y...
                ||  Nodes(Y).parent ~= X &&  Nodes(Y).h > Nodes(X).h + cost_X_Y && Y ~= goalPos
            % 若情况1，表明邻节点Y还未纳入openList，那么就以X作为父节点；
            % 若情况2，表明虽然Y的父节点是X，但是 Nodes(Y).h却与Nodes(X).h + cost_X_Y不相等了，
            % 表明Y的父节点Nodes(X).h 有过更新，可能是由于障碍引起的；
            % 若情况3，表明Y可以通过将X作为父节点，使得Nodes(Y).h更小
            % 上述情况，都应该将X作为Y的父节点，并把Y移到openList后，再进一步考察
            Nodes(Y).parent = X;
            h_new = Nodes(X).h + cost_X_Y;
            [Nodes,openList] = insert(Nodes,openList,Y,h_new);
        end

    end
   

   
else 
    % 若k_old ~= Nodes(X).h,Raise态
    % 说明节点X受到了影响，遍历其邻域。
    
    for i = 1:size(neighborNodes,1)
        Y = neighborNodes(i,1);
        cost_X_Y = neighborNodes(i,2);
        if  isequal(Nodes(Y).t, 'new')||...
                Nodes(Y).parent == X && Nodes(Y).h ~= Nodes(X).h + cost_X_Y
            % 若情况1，表明邻节点Y还未纳入openList，那么就以X作为父节点；
            % 若情况2，表明虽然Y的父节点是X，但是 Nodes(Y).h却与Nodes(X).h + cost_X_Y不相等了，
            % 上述情况，都应该将X作为Y的父节点，并把Y移到openList后，再进一步考察
            Nodes(Y).parent = X;
            h_new = Nodes(X).h + cost_X_Y;
            [Nodes,openList] = insert(Nodes,openList,Y,h_new);
        else

            if  Nodes(Y).parent ~= X && Nodes(Y).h > Nodes(X).h + cost_X_Y
                % 若满足，表明Y可以通过将X作为父节点，使得Nodes(Y).h更小                
                % 但是目前针对X，有 k_old ~= Nodes(X).h，故要将X追加到openList，待下一次循环满足条件就将X作为Y的父节点。
                [Nodes,openList] = insert(Nodes,openList,X,Nodes(X).h);
            elseif  Nodes(Y).parent ~= X && Nodes(X).h > Nodes(Y).h + cost_X_Y...
                    &&  isequal(Nodes(Y).t, 'closed') && Nodes(Y).h > k_old
               % 情况1，表明Y的父节点不是X，但是让Y成为X父节点， Nodes(X).h更小；
               % 情况2，表明并且Y已经被open表移除；
               % 情况3，当前从openList取出的最小值k_old居然比h(Y)小，表明已经被移除open表的Y受到了障碍影响导致h值升高
               % 上述情况，都要重新将Y置于openList中，进行下一轮考察。              
                [Nodes,openList] = insert(Nodes,openList,Y,Nodes(Y).h);
            end
        end
    end
end

end

function [Nodes, U] = computeShortestPath(field, Nodes, U, km,rows,cols,startPos,goalPos)

while true
    
    % 对U按照先k1，后k2排序
    U = sortrows(U,[2 3]);
    u = U(1,1);
    k_old = U(1,2);
    k_u = calculateKey(Nodes(u),startPos, km, rows, cols);
    U(1,:) = [];
    
    % 如果找到了机器人当前位置点，或者U(1,2)>calculateKey(startPos),退出
    k_start = calculateKey(Nodes(startPos),startPos, km, rows, cols);
    if k_old > k_start(1)
        break
    end
    if ~isinf(Nodes(startPos).rhs) && Nodes(startPos).rhs == Nodes(startPos).g
        break
    end
    
    if k_old < k_u(1)
        % 若满足，表明节点u的k值做过修改，即受到了环境的影响（如突增障碍物）
        % 故应当重新插入到U队列中，做进一步的考察
        U(end+1,:) = [u,k_u];
        
    elseif Nodes(u).g > Nodes(u).rhs
        % 若满足，局部过一致状态，表明节点u有新的捷径
        % 那么就考察u的邻近节点，判断以u作为父节点是否路径更优，并更新
        Nodes(u).g = Nodes(u).rhs;
        neighborNodes = getNeighborNode(field, u);
        for i = 1:length(neighborNodes)
            neighborNode = neighborNodes(i);
            [Nodes,U] = updateVertex(field,Nodes,U,neighborNode, km,rows,cols,startPos,goalPos);
        end
    else
        % 若上述都不满足，表明遇到障碍物，局部欠一致状态
         Nodes(u).g = inf;
         
         % 更新u的邻近节点
         neighborNodes = getNeighborNode(field, u);
         influencedNnodes = [neighborNodes,u];
         for i = 1:length(influencedNnodes)
             s = influencedNnodes(i);
             [Nodes,U] = updateVertex(field,Nodes,U,s, km,rows,cols,startPos,goalPos);
         end
    end
end

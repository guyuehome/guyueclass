function [Nodes, U] = computeShortestPath(field, Nodes, U, rows,cols,startPos,goalPos)

% 这里参照A*算法定义了closeList集合，用于排除部分邻节点
closeList = [];
while true
    
    % 对U按照先k1，后k2排序
    U = sortrows(U,[2 3]);
    
    % 如果找到了目标点，或者U(1,2)>calculateKey(goalPos),退出循环
    if U(1,2) > calculateKey(Nodes(U(1,1)),goalPos, rows, cols)
        break
    end
    if ~isinf(Nodes(goalPos).rhs) && Nodes(goalPos).rhs == Nodes(goalPos).g
        break
    end
    
    
    node = U(1,1);
    closeList(end+1) = node;
    if Nodes(node).g > Nodes(node).rhs   % 局部过一致
        
        Nodes(node).g =   Nodes(node).rhs;
        [Nodes,U] = updateVertex(field,Nodes,U,node,rows,cols,startPos,goalPos);
        
        % 找到当前节点的所有邻近节点，除开位于closeList中的节点
        neighborNodes = getNeighborNode(field,closeList, node);
        
        % 更新所有子节点的rhs
        for i = 1:length(neighborNodes)
            neighborNode = neighborNodes(i);
            [Nodes,U] = updateVertex(field,Nodes,U,neighborNode,rows,cols,startPos,goalPos);
        end  
        
    else  % 局部一致或局部欠一致
        Nodes(node).g = inf;
        [Nodes,U] = updateVertex(field,Nodes,U,node,rows,cols,startPos,goalPos);
        
        % 找到当前节点的所有邻近子节点，除开位于closeList中的节点
        neighborNodes = getNeighborNode(field,closeList, node);
        
        % 更新所有子节点的rhs
        for i = 1:length(neighborNodes)
            neighborNode = neighborNodes(i);
            [Nodes,U] = updateVertex(field,Nodes,U,neighborNode,rows,cols,startPos,goalPos);
        end
    end
 
end
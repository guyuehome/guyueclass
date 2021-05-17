function [Nodes,U] = updateVertex(field,Nodes,U,node,rows,cols,startPos,goalPos)
if node ~= startPos
    Nodes(node).rhs = inf;
    [node_sub(1), node_sub(2)] = ind2sub([rows,cols],node);
    
    % 找到当前节点的周围所有邻近节点
    neigborNodes = getNeighborNode(field,[], node);
    
    % 遍历计算邻近节点的g(s')+C(s',u)
    value = [];
    for i = 1:length(neigborNodes)
        neighborNode = neigborNodes(i);
        [neighborNode_sub(1), neighborNode_sub(2)] = ind2sub([rows,cols],neighborNode);
        cost = norm(neighborNode_sub - node_sub);
        value(i) = Nodes(neighborNode).g + cost; 
    end
    
    % 获取周围g(s')+C(s',u)的值最小的邻近节点，赋值给rhs(u)，并更新父节点
     [Nodes(node).rhs, idx] = min(value);
     Nodes(node).parent = neigborNodes(idx); 
end   

% 如果node已经存在于U，移出
[in,idx] = ismember(node, U(:,1));
if in
    U(idx,:) = [];
end

% 如果node的rhs和g不相等，则添加到U中
if Nodes(node).rhs ~= Nodes(node).g
    U(end+1,1) = node;
    U(end,2:3) = calculateKey(Nodes(node),goalPos, rows, cols);
end

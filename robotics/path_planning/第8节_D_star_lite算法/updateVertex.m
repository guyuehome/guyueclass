function [Nodes,U] = updateVertex(field,Nodes,U,u, km,rows,cols,s_start,goalPos)
if u ~= goalPos
    Nodes(u).rhs = inf;
    [node_sub(1), node_sub(2)] = ind2sub([rows,cols],u);
    
    % 找到当前节点的周围所有邻近节点
    neigborNodes = getNeighborNode(field, u);
    
    % 遍历计算邻近节点的g(s')+C(s',u)
    value = [];
    for i = 1:length(neigborNodes)
        neighborNode = neigborNodes(i);
        [neighborNode_sub(1), neighborNode_sub(2)] = ind2sub([rows,cols],neighborNode);
        cost = norm(neighborNode_sub - node_sub);
        value(end+1) = Nodes(neighborNode).g + cost;
    end
    
    % 获取周围g(s')+C(s',u)的值最小的邻近节点，赋值给rhs(u)，并更新父节点
    [Nodes(u).rhs, idx] = min(value);
    Nodes(u).parent = neigborNodes(idx);
end

% 如果node已经存在于queue，移出
[in,idx] = ismember(u, U(:,1));
if in
    U(idx,:) = [];
end

% 如果node的rhs和g不相等，则添加到queue中
if Nodes(u).rhs ~= Nodes(u).g
    U(end+1,1) = u;
    U(end,2:3) = calculateKey(Nodes(u),s_start, km, rows, cols);
end

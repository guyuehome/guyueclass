function influencedChildNnodes = findInfluencedNnodes(field,Nodes,obsNodes)
[rows, cols] = size(field);
influencedChildNnodes = [];         % 受影响的子节点
influencedParentNnodes = obsNodes;  % 受影响的父节点
while true
    influencedChildNnode = [];
    for i = 1:length(influencedParentNnodes)
        % 针对每一个受影响父节点，找到他的邻近节点
        % 判断这个邻近节点是否受到了影响
        
        influencedParentNnode = influencedParentNnodes(i);
        childNodes = getNeighborNode(field,[], influencedParentNnode);
        for j = 1:length(childNodes)
            childNode = childNodes(j);
            [childNode_sub(1), childNode_sub(2)] = ind2sub([rows,cols],childNode);
            parentNodes = getNeighborNode(field,[], childNode);
            value = [];
            for k = 1:length(parentNodes)
                parentNode = parentNodes(k);
                [parentNode_sub(1), parentNode_sub(2)] = ind2sub([rows,cols],parentNode);
                cost = norm(parentNode_sub - childNode_sub);
                g = Nodes(parentNode).g;
                value(k) = cost + g;
            end
            
            % 获得节点childNode的最小父节点
            [~,idx] = min(value);
            node = parentNodes(idx);
            
            if node == influencedParentNnode
                influencedChildNnode(end+1) = childNode;
            end
        end
    end
    
    influencedChildNnodes = [influencedChildNnodes, influencedChildNnode];
    influencedParentNnodes = influencedChildNnode;
    if isempty(influencedParentNnodes)
        break
    end
    
end

% influencedChildNnodes中某些节点可能多次出现
% 用unique函数删掉重复出现的点
influencedChildNnodes = unique(influencedChildNnodes);
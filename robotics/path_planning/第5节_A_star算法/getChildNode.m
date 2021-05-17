function childNodes = getChildNode(field,closeList, parentNode)
% 选取父节点周边8个节点作为备选子节点，线性化坐标
% 排除超过边界之外的、位于障碍区的、位于closeList中的

[rows, cols] = size(field);
[row_parentNode, col_parentNode] = ind2sub([rows, cols], parentNode);
childNodes = [];
closeList = closeList(:,1);

% 第1个子节点
childNode = [row_parentNode, col_parentNode+1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end

% 第2个子节点
childNode = [row_parentNode-1, col_parentNode+1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% 第3个子节点
childNode = [row_parentNode-1, col_parentNode];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% 第4个子节点
childNode = [row_parentNode-1, col_parentNode-1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% 第5个子节点
childNode = [row_parentNode, col_parentNode-1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% 第6个子节点
childNode = [row_parentNode+1, col_parentNode-1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% 第7个子节点
childNode = [row_parentNode+1, col_parentNode];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% 第8个子节点
childNode = [row_parentNode+1, col_parentNode+1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end





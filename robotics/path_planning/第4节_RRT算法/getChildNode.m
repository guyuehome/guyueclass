function childNode = getChildNode(field, parentNode, samplePoint)
% 定义生长单步长为2个栅格，选取父节点周边16个节点作为备选子节点
% 根据随机采样点与父节点的角度，确定生长的子节点


[rows, cols] = size(field);
[row_samplePoint, col_samplePoint] = ind2sub([rows, cols], samplePoint);
[row_parentNode, col_parentNode] = ind2sub([rows, cols], parentNode);


% 定义16个点的行列坐标
% 注意，为了行列坐标与x/y坐标匹配，从父节点的下节点逆时针开始定义，依次编号
childNode_set = [ row_parentNode+2, col_parentNode;
    row_parentNode+2, col_parentNode+1;
    row_parentNode+2, col_parentNode+2;
    row_parentNode+1, col_parentNode+2;
    row_parentNode, col_parentNode+2; 
    row_parentNode-1, col_parentNode+2;
    row_parentNode-2, col_parentNode+2;
    row_parentNode-2, col_parentNode+1;
    row_parentNode-2, col_parentNode;
    row_parentNode-2, col_parentNode-1;
    row_parentNode-2, col_parentNode-2;
    row_parentNode-1, col_parentNode-2;
    row_parentNode,   col_parentNode-2;
    row_parentNode+1, col_parentNode-2;
    row_parentNode+2, col_parentNode-2;
    row_parentNode+2, col_parentNode-1];

% 计算16个子节点的角度范围集，和当前随机点的角度范围
theta_set = linspace(0,2*pi,16);
theta = atan2((col_samplePoint - col_parentNode), ...
    (row_samplePoint - row_parentNode));

% 若theta位于第三四象限，加上2*pi
if theta < 0
    theta = theta + 2*pi;
end

% 遍历周围的16个点，判断角度位于哪一个范围
for i = 1:15
    if theta >= theta_set(i) && theta < theta_set(i+1)
        childNodeIdx = i;
        break
    end
end

% 选中的子节点
childNode = childNode_set(childNodeIdx,:);



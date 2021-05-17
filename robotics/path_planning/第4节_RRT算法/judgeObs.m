function flag = judgeObs(field, parentNode, childNode)

flag = 0;
[rows, cols] = size(field);

% 判断子节点是否在障碍物上
obsIdx = find(field == 2);
if ismember(childNode, obsIdx)
    flag = 1;
    return
end

% 判断父节点与子节点的连线是否跨过障碍物
[parentNode(1), parentNode(2)] = ind2sub([rows, cols], parentNode);
[childNode(1), childNode(2)] = ind2sub([rows, cols], childNode);

P2 = parentNode;
P1 = childNode;
row_min = min([P1(1), P2(1)]);
row_max = max([P1(1), P2(1)]);
col_min = min([P1(2), P2(2)]);
col_max = max([P1(2), P2(2)]);
for row = row_min:row_max
    for col = col_min:col_max
        if field(row, col) == 2
            P = [row, col];
            
            % 直接计算障碍物节点距P1和P2构成的连线的距离
            d = abs(det([P2-P1;P-P1]))/norm(P2-P1);
            if d < 0.5
                flag = 1;
                return
            end
        end
    end
end
            

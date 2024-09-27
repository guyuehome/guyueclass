function childPop = mutation(childPop,p_mut,posBound)
% 获取父代种群数、染色体长度
m = size(childPop,1);
n = length(childPop(1).pos.x);
for i = 1:1:m
    if rand < p_mut
        idx = round(rand*n);
        
        % 避免越界
        if idx <= 1
            idx = 2;
        end
        if idx == n
            idx = n-1;
        end
        
        % 变异：随机数替换
        childPop(i).pos.x(idx) = rand*(posBound(1,2)-posBound(1,1)) + posBound(1,1);
        childPop(i).pos.y(idx) = rand*(posBound(2,2)-posBound(2,1)) + posBound(2,1);
        childPop(i).pos.z(idx) = rand*(posBound(3,2)-posBound(3,1)) + posBound(3,1);
    end
end

% 将所有控制点按照x/y/z三个方向进行排序
for i = 1:m
    childPop(i).pos.x = sort(childPop(i).pos.x);
    childPop(i).pos.y = sort(childPop(i).pos.y);
    childPop(i).pos.z = sort(childPop(i).pos.z);
end

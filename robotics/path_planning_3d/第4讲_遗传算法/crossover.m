function childPop = crossover(parentPop,p_crs)

% 获取父代种群数、染色体长度
m = size(parentPop,1);
n = length(parentPop(1).pos.x);

% 将parentPop赋值给childPop，以初始化子代种群
childPop = parentPop;

% 交叉操作
for i = 1:2:m-1
    if rand < p_crs
        idx = round(rand*n);
        childPop(i).pos.x = [parentPop(i).pos.x(1:idx), parentPop(i+1).pos.x(idx+1:n)];
        childPop(i+1).pos.x = [parentPop(i+1).pos.x(1:idx), parentPop(i).pos.x(idx+1:n)];
        childPop(i).pos.y = [parentPop(i).pos.y(1:idx), parentPop(i+1).pos.y(idx+1:n)];
        childPop(i+1).pos.y = [parentPop(i+1).pos.y(1:idx), parentPop(i).pos.y(idx+1:n)];
        childPop(i).pos.z = [parentPop(i).pos.z(1:idx), parentPop(i+1).pos.z(idx+1:n)];
    end
end

% 将所有控制点按照x/y/z三个方向进行排序
for i = 1:m
    childPop(i).pos.x = sort(childPop(i).pos.x);
    childPop(i).pos.y = sort(childPop(i).pos.y);
    childPop(i).pos.z = sort(childPop(i).pos.z);
end

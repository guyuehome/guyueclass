function pop = initPop(popNum,chromLength,posBound)
pop = struct;

% 第一代的个体初始化
for i = 1:popNum
    % 初始化
    pop(i).pos= [];
    pop(i).fitness = [];
    pop(i).path = [];
    pop(i).Best.pos = [];
    pop(i).Best.fitness = inf;
    pop(i).Best.path = [];
    
    % 随机生成初始控制点（染色体）
    pop(i).pos.x = (posBound(1,2)-posBound(1,1)) * rand(1,chromLength) + posBound(1,1);
    pop(i).pos.y = (posBound(2,2)-posBound(2,1)) * rand(1,chromLength) + posBound(2,1);
    pop(i).pos.z = (posBound(3,2)-posBound(3,1)) * rand(1,chromLength) + posBound(3,1);
end

% 将所有控制点按照x/y/z三个方向进行排序
for i = 1:popNum
    pop(i).pos.x = sort(pop(i).pos.x);
    pop(i).pos.y = sort(pop(i).pos.y);
    pop(i).pos.z = sort(pop(i).pos.z);
end
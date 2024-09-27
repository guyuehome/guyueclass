function parentPop = select(pop, p_select)

% 利用轮盘赌法执行选择操作
fit_reverse = 1./[pop.fitness]'; 
totalFit = sum(fit_reverse);
accP = cumsum(fit_reverse/totalFit);        % 概率累计和
selectNum = round(size(pop,2) * p_select);  % 选择的个体数量

% 将pop的第一行赋值给parentPop，以实现初始化
parentPop = pop(1);
for i=1:selectNum
    % 找到比随机数大的累积概率
    idx = find(accP>rand); 
    if isempty(idx)
        continue
    end  
    
    % 将首个比随机数大的累积概率的位置的个体遗传下去
    parentPop(i) = pop(idx(1));
end


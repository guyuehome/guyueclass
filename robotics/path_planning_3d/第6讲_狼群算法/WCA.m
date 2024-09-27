% 第6讲：狼群算法
% 作者： Ally
% 日期： 2021/09/24
clc
clear
close all

%% 三维路径规划模型定义
startPos = [1, 1, 1];
goalPos = [100, 100, 80];

% 随机定义山峰地图
mapRange = [100,100,100];              % 地图长、宽、高范围
[X,Y,Z] = defMap(mapRange);

%% 初始参数设置
iterMax = 100;              % 迭代次数
pointNum = 3;               % 每一个粒子包含三个位置点
N = 100;                    % 狼群总数
alpha = 0.5;                % 探狼比例因子
S_num = N * alpha;          % 探狼数量
M_num = N - S_num ;         % 猛狼数量
S = 20;                     % 步长因子
step_a = mapRange / S;      % 游走步长
step_b = 2*step_a;          % 奔袭步长
step_c = step_a/2;          % 围捕步长
h = 20;                     % 游走方向数
d0 = 10;                    % 由奔袭转围攻的临界距离
T_max = 10;                 % 围攻最大次数
posBound = [[0,0,0]',mapRange'];   % 位置界限

%% 种群初始化
% 初始化一个空的粒子结构体
wolf = struct;
wolf.pos= [];
wolf.fitness = [];
wolf.path = [];
wolfs = repmat(wolf,N,1);
wolfs_s = repmat(wolf,S_num,1);
wolfs_m = repmat(wolf,M_num,1);

% 初始化每一代的最优粒子
wolf_header.fitness = inf;

% 狼群随机分布位置
for i = 1:N
    % 粒子按照正态分布随机生成
    wolfs(i).pos.x = unifrnd(posBound(1,1),posBound(1,2),1,pointNum);
    wolfs(i).pos.y = unifrnd(posBound(2,1),posBound(2,2),1,pointNum);
    wolfs(i).pos.z = unifrnd(posBound(3,1),posBound(3,2),1,pointNum);
    
    % 适应度
    [flag,fitness,path] = calFitness(startPos, goalPos,X,Y,Z, wolfs(i).pos);
    
    % 碰撞检测判断
    if flag == 1
        % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
        wolfs(i).fitness = 1000*fitness;
        wolfs(i).path = path;
    else
        % 否则，表明可以选择此路径
        wolfs(i).fitness = fitness;
        wolfs(i).path = path;
    end
    
    % 更新全局最优
    if wolfs(i).fitness < wolf_header.fitness
        wolf_header = wolfs(i);
        idx = i;
    end
end

% 选择其中适应度最小的S_num只狼作为探狼，其余作为猛狼
[~,idx_sort] = sort([wolfs.fitness]);
for i = 1:S_num
    wolfs_s(i)= wolfs(idx_sort(i));
end
for i = S_num+1 : N
    wolfs_m(i-S_num)= wolfs(idx_sort(i));
end

% 初始化每一代的最优适应度，用于画适应度迭代图
fitness_beat_iters = zeros(iterMax,1);


%% 循环
for iter = 1:iterMax
    % 1.游走行为：探狼游走
    flag_exit = 0;
    for i = 1:S_num
        
        % 第i只探狼向四周一共h个方向进行游走搜索
        pos_k = struct('x',[],'y',[],'z',[]);
        path_k = cell(0);
        fitness_k= [];
        for k = 1:h
            % 游走的新位置
            pos_new.x = wolfs_s(i).pos.x + step_a * sin(2*pi*k/h);
            pos_new.y = wolfs_s(i).pos.y + step_a * sin(2*pi*k/h);
            pos_new.z = wolfs_s(i).pos.z + step_a * sin(2*pi*k/h);
            
            % 判断是否位于位置界限以内
            pos_new.x = max(pos_new.x, posBound(1,1));
            pos_new.x = min(pos_new.x, posBound(1,2));
            pos_new.y = max(pos_new.y, posBound(2,1));
            pos_new.y = min(pos_new.y, posBound(2,2));
            pos_new.z = max(pos_new.z, posBound(3,1));
            pos_new.z = min(pos_new.z, posBound(3,2));
            
            % 适应度计算
            [flag,fitness,path ] = calFitness(startPos, goalPos,X,Y,Z, pos_new);
            
            % 碰撞检测判断
            if flag == 1
                % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
                fitness = 1000*fitness;
            end
            
            % 将每一个方向的位置、路径和适应度保存
            pos_k(k) = pos_new;
            path_k{k,1} = path;
            fitness_k(k,1) = fitness;
        end
        
        
        % h个方向的最优适应度与本探狼当前位置的适应度进行比较
        [fitness_min,idx] = min(fitness_k);
        if fitness_min < wolfs_s(i).fitness
            wolfs_s(i).pos = pos_k(idx);
            wolfs_s(i).path = path_k(idx);
            wolfs_s(i).fitness = fitness_k(idx);
            
            % 与头狼位置的适应度进行比较,若适应度更小，则更换头狼位置，并退出大循环
            if wolfs_s(i).fitness < wolf_header.fitness
                wolf_header = wolfs_s(i);
                flag_exit = 1;
                break
            end
        end
        
        % 判断是否发出召唤行为
        if flag_exit == 1
            break
        end
        
    end
      
    % 2.召唤行为：猛狼开始朝头狼位置奔袭
    flag_exit = 0;
    for i = 1:M_num        
        while flag_exit == 0           
            % 猛狼奔袭
            % 第i只猛狼向头狼位置奔袭
            pos_header = [wolf_header.pos.x; wolf_header.pos.y; wolf_header.pos.z];
            pos_now = [wolfs_m(i).pos.x; wolfs_m(i).pos.y; wolfs_m(i).pos.z];
            for j = 1:3
                pos_next(:,j) = pos_now(:,j) + step_b' .* (pos_header(:,j) - pos_now(:,j))/norm(pos_header(:,j) - pos_now(:,j));
            end
            wolfs_m(i).pos.x = pos_next(1,:);
            wolfs_m(i).pos.y = pos_next(2,:);
            wolfs_m(i).pos.z = pos_next(3,:);
            
            % 判断是否位于位置界限以内
            wolfs_m(i).pos.x = max(wolfs_m(i).pos.x, posBound(1,1));
            wolfs_m(i).pos.x = min(wolfs_m(i).pos.x, posBound(1,2));
            wolfs_m(i).pos.y = max(wolfs_m(i).pos.y, posBound(2,1));
            wolfs_m(i).pos.y = min(wolfs_m(i).pos.y, posBound(2,2));
            wolfs_m(i).pos.z = max(wolfs_m(i).pos.z, posBound(3,1));
            wolfs_m(i).pos.z = min(wolfs_m(i).pos.z, posBound(3,2));
            
            % 适应度计算
            [flag,fitness, wolfs_m(i).path] = calFitness(startPos, goalPos,X,Y,Z, wolfs_m(i).pos);
            
            % 碰撞检测判断
            if flag == 1
                % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
                wolfs_m(i).fitness = 1000*fitness;
            else
                % 否则，表明可以选择此路径
                wolfs_m(i).fitness = fitness;
            end
            
            % 与头狼位置的适应度进行比较,若适应度更小，则更换头狼位置，重新执行召唤行为
            if wolfs_m(i).fitness < wolf_header.fitness
                wolf_header = wolfs_m(i);
                break
            end
            
            % 判断当前位置是否已经到达围攻临界距离
            d = sqrt((pos_next - pos_header).^2);
            % 3. 若d < d0，进入围捕行为
            if d < d0
                
                T = 0; % 围捕累计次数
                while T < T_max

                    % 执行围捕动作时位置的更新
                    pos_header = [wolf_header.pos.x; wolf_header.pos.y; wolf_header.pos.z];
                    pos_now = [wolfs_m(i).pos.x; wolfs_m(i).pos.y; wolfs_m(i).pos.z];
                    for j = 1:3
                        pos_next(:,j) = pos_now(:,j) + step_c'*rand .* (pos_header(:,j) - pos_now(:,j))/norm(pos_header(:,j) - pos_now(:,j));
                    end
                    
                    wolfs_m(i).pos.x = pos_next(1,:);
                    wolfs_m(i).pos.y = pos_next(2,:);
                    wolfs_m(i).pos.z = pos_next(3,:);
                    
                    % 适应度计算
                    [flag,fitness,wolfs_m(i).path] = calFitness(startPos, goalPos,X,Y,Z, wolfs_m(i).pos);
                    
                    % 碰撞检测判断
                    if flag == 1
                        wolfs_m(i).fitness = 1000*fitness;
                    else
                        wolfs_m(i).fitness = fitness;
                    end
                    
                    % 与头狼位置的适应度进行比较,若适应度更小，则更换头狼位置，重新执行召唤行为
                    if wolfs_m(i).fitness < wolf_header.fitness
                        wolf_header = wolfs_m(i);
                        flag_exit = 1;
                        break
                    end
                    
                    T = T + 1;
                end
            end
        end
    end
    
    % 狼群更新：优胜劣汰
    wolfs = [wolfs_s; wolfs_m];
    [~,idx_sort] = sort([wolfs.fitness]);
    
    % 将排名前S_num只狼作为探狼，后面的舍掉，并随机生成新的猛狼位置
    for i = 1:S_num
        wolfs_s(i)= wolfs(idx_sort(i));
    end
    for i = 1:M_num
        % 粒子按照正态分布随机生成
        wolfs_m(i).pos.x = unifrnd(posBound(1,1),posBound(1,2),1,pointNum);
        wolfs_m(i).pos.y = unifrnd(posBound(2,1),posBound(2,2),1,pointNum);
        wolfs_m(i).pos.z = unifrnd(posBound(3,1),posBound(3,2),1,pointNum);
        
        % 适应度
        [flag,fitness,wolfs_m(i).path] = calFitness(startPos, goalPos,X,Y,Z, wolfs_m(i).pos);
        
        % 碰撞检测判断
        if flag == 1
            % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
            wolfs_m(i).fitness = 1000*fitness;
        else
            % 否则，表明可以选择此路径
            wolfs_m(i).fitness = fitness;
        end
        
        % 更新全局最优
        if wolfs_m(i).fitness < wolf_header.fitness
            wolf_header = wolfs_m(i);
        end
    end
    
    % 把每一代的最优粒子赋值给fitness_beat_iters
    fitness_beat_iters(iter) = wolf_header.fitness;
    
    % 在命令行窗口显示每一代的信息
    disp(['第' num2str(iter) '代:' '最优适应度 = ' num2str(fitness_beat_iters(iter))]);
    
    % 画图
    plotFigure(startPos,goalPos,X,Y,Z,wolf_header);
    pause(0.001);
end

%% 结果展示
% 理论最小适应度：直线距离
fitness_best = norm(startPos - goalPos);
disp([ '理论最优适应度 = ' num2str(fitness_best)]);

% 画适应度迭代图
figure
plot(fitness_beat_iters,'LineWidth',2);
xlabel('迭代次数');
ylabel('最优适应度');

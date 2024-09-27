classdef nsga2 < handle
    %用于实现多目标遗传算法
    properties
        ranges              %变量取值边界范围
        Gm                  %代数
        Np                  %种群个体数
        D                   %变量数
        M                   %目标变量个数
        ObjFcn              %目标函数
        
        F0 = 0.1;           %突变因子
        M0 = 0.4;           %突变比例
        CR0 = 0.5;          %交叉比例

        XG                  %种群个体
        saveG = 5;
        saveSample = false;
        folderName='CalResults';%存放计算结果的文件夹名称
    end
    properties(SetAccess = private)
        ObjValue            %目标的值
    end

    methods
        function obj = nsga2(xmin,xmax,Gm,Np,M,ObjFcn,F0,M0,CR0)
            %构造函数
            obj.ranges = [xmin',xmax'];
            obj.Gm = Gm;
            obj.Np = Np;
            obj.D = size(obj.ranges,1);
            obj.M = M;
            obj.ObjFcn = ObjFcn;
            
            if nargin == 9
                obj.F0 = F0;
                obj.M0 = M0;
                obj.CR0 = CR0;
            end
        end
        
        function initialpopulation(obj)
            %种群初始化
            fprintf('正在进行初始种群的计算。\n');
            obj.XG = LHS(obj.ranges,obj.Np);
            obj.update;
            fprintf('初始种群计算完成。\n');
            
            %用于保存计算结果
            %if ~exist(obj.folderName,'dir')
            %exist会搜索所有已添加的路径，不仅限于当前路径下，所以替换为isfolder
            if ~isfolder(obj.folderName)
                mkdir(obj.folderName);
            end
        end
        
        function update(obj, indexs)
            %更新目标函数的值
            %indexs为需要更新的个体标号
            if nargin == 1
                indexs = 1:size(obj.XG,1);
            end
            obj.ObjValue(indexs,:) = obj.ObjFcn(obj.XG(indexs,:));
        end
        
        function mu_index = mutation(obj)
            %执行变异操作
            mu_index = randsample(obj.Np,round(obj.M0*obj.Np));%待变异的个体编号
            range_diff = repmat([obj.ranges(:,2)-obj.ranges(:,1)]',length(mu_index),1);
            obj.XG(mu_index,:) = obj.XG(mu_index,:) + obj.F0*randn(length(mu_index),obj.D).*range_diff;
        end
        
        function constraint(obj)
            %对超出约束范围的个体重新取值
            for i=1:obj.Np
                up=obj.XG(i,:)>obj.ranges(:,2)';
                obj.XG(i,up)=obj.ranges(up,2)';
                down=obj.XG(i,:)<obj.ranges(:,1)';
                obj.XG(i,down)=obj.ranges(down,1)';
            end
        end

        function cr_index = crossover(obj)
            %执行交叉操作
            cr_n=2*round(obj.CR0*obj.Np/2);%交叉数量
            cr_index=randsample(obj.Np,cr_n);%待交叉个体标号
            cr1=cr_index(1:cr_n/2);
            cr2=cr_index(cr_n/2+1:end);
            [obj.XG(cr1,:),obj.XG(cr2,:)]= ...
                DoCross(obj.XG(cr1,:),obj.XG(cr2,:));
        end
        
        function reproduce(obj)
            %种群繁衍新生
            XG_last = obj.XG;
            ObjValue_last = obj.ObjValue;
            
            mu_index = obj.mutation;%执行变异操作
            obj.constraint;
            cr_index = obj.crossover;%执行交叉操作
            changed_index = union(mu_index, cr_index);
            obj.update(changed_index);

            obj.XG=[XG_last;obj.XG(changed_index,:)];
            obj.ObjValue=[ObjValue_last;obj.ObjValue(changed_index,:)];
        end
        
        function fronts = fronts_cal(obj)
            %个体排序分层
            N = size(obj.XG,1);%种群的数量
            individual = zeros(N);%第i行第j列置1表示i支配j
            for i = 1:N
                for j = 1:N
                    if all(obj.ObjValue(i,:)<=obj.ObjValue(j,:)) && ...
                            any(obj.ObjValue(i,:)<obj.ObjValue(j,:))
                        individual(i,j) = 1;
                    end
                end
            end
            
            front = 1;
            left = 1:N;       %待排序的个体
            fronts = ones(N,1);
            while ~isempty(left)
                F = [];
                for i=left
                    if ~any(individual(left,i))
                        F=[F,i];
                        fronts(i) = front;
                    end
                end
                left=setdiff(left,F);        %求差集
                front=front+1;
            end
        end

        function levels = distances_cal(obj, fronts)
            %计算拥挤度
            N = size(obj.XG,1);%种群的数量
            distances=inf*ones(N,obj.M);
            for front = 1:max(fronts)
                f_index=find(fronts==front);
                y=obj.ObjValue(f_index,:);    %y中存放排序等级为front的矩阵
                
                %基于拥挤距离的矩阵
                for i = 1:obj.M
                    %按照目标函数值排序，其中index_obj为索引
                    [sorted_obj,index_obj]=sortrows(y,i);
                    %使用bounds函数同时获取目标函数的最大值和最小值
                    [fmin,fmax]=bounds(y(:,i));

                    for j = 2:size(y,1)-1     %从第二个个体计算拥挤距离
                        %第一个个体和最后一个个体保持inf
                        next_obj = sorted_obj(j+1,i);
                        previous_obj = sorted_obj(j-1,i);
                        if fmax ~= fmin
                            distances(f_index(index_obj(j)),i) = (next_obj-previous_obj)/(fmax-fmin);
                        end
                    end
                end
            end
            levels=1./sum(distances,2);%拥挤度设置为拥挤距离的倒数
        end

        function f1 = selection(obj)
            %个体选择适者生存
            fronts = fronts_cal(obj);
            levels = distances_cal(obj, fronts);
            [~,index] = sortrows([fronts,levels],[1 2]);
            obj.XG = obj.XG(index(1:obj.Np),:);
            obj.ObjValue = obj.ObjValue(index(1:obj.Np),:);
            f1 = sum(fronts(index(1:obj.Np))==1);
        end
        
        function train(obj)
            %对种群进行指定代数的训练
            SampleData.XG = obj.XG;
            SampleData.ObjValue = obj.ObjValue;
            %开始计时
            tic;
            for G=1:obj.Gm
                fprintf('正在计算第%d代，共%d代。\n',G,obj.Gm);
                obj.reproduce;
                f1 = obj.selection;
                if obj.saveSample
                    SampleData.XG = [SampleData.XG;obj.XG];
                    SampleData.ObjValue = [SampleData.ObjValue;obj.ObjValue];
                end
                t=toc;
                fprintf('第%d代计算完成，已用时长%f分钟。\n',G,t/60);
                fprintf('当前F1个体数量：%d。\n',f1);
                
                %保存对象
                if mod(G,obj.saveG)==0 || G==obj.Gm
                    save([obj.folderName,'/nsga2_obj.mat'],'obj');
                    if obj.saveSample
                        save([obj.folderName,'/SampleData.mat'],'SampleData');
                    end
                end
            end
        end
        
        function plotCosts(obj,picname,cols)
            %绘制目标
            if nargin < 3
                cols = [1,2];
            end
            if nargin < 2
                picname = [];
            end
            scatter(obj.ObjValue(:,cols(1)),obj.ObjValue(:,cols(2)),'filled');
            xlabel('1^{st} Objective');
            ylabel('2^{nd} Objective');
            title('Non-dominated Solutions');
            grid on;
            set(gca,'fontsize',12);
            if picname
                print(gcf,[obj.folderName,'/',picname],'-djpeg','-r600');
            end
        end
        
        function writeLog(obj)
            %写日志文件
            fid=fopen([obj.folderName,'/nsga2.log'],'w');
            fprintf(fid,'共%d代，每代%d个个体。\n',obj.Gm,obj.Np);
            fprintf(fid,['计算完成时间：',datestr(now+1/3),'\n']);
            fclose(fid);
        end
    end
end

function [y1, y2]=DoCross(x1,x2)
%交叉操作
%x1，x2：待交叉的两个个体
%y1，y2：交叉后的两个个体
alpha=rand(size(x1));
y1=alpha.*x1+(1-alpha).*x2;
y2=alpha.*x2+(1-alpha).*x1;
end
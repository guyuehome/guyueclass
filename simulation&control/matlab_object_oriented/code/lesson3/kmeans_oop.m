classdef kmeans_oop < handle
    properties
        D   %输入数据
        k   %聚类类别数
        n   %输入数据样本个数
        c   %聚类类别标号
        center  %中心点坐标
    end

    methods(Static)
        function c=find_nearset(p,points)
            %寻找points中哪个和p最近，返回索引
            dist=inf;
            for i=1:size(points,1)
                if norm(p-points(i,:))<dist
                    dist=norm(p-points(i,:));
                    c=i;
                end
            end
        end
    end

    methods
        function obj = kmeans_oop(D,k)
            obj.D = D;
            obj.k = k;
        end

        function init(obj)  %初始化
            obj.n = size(obj.D,1);
            obj.c=zeros(obj.n,1);
            seed=randsample(obj.n,obj.k);
            obj.center=obj.D(seed,:);
        end

        function update_c(obj)  %更新类别
            for i=1:obj.n
                obj.c(i)=obj.find_nearset(obj.D(i,:),obj.center);
            end
        end

        function updata_center(obj) %更新中心点
            for i=1:obj.k
                obj.center(i,:)=mean(obj.D(obj.c==i,:),1);
            end
        end

        function train(obj) %执行训练
            while true
                c_last=obj.c;
                obj.update_c;
                if all(obj.c==c_last)
                    break
                end
                obj.updata_center;
            end
        end
    end
end
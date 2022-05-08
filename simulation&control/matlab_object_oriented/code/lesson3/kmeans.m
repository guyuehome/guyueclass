function c=kmeans(D,k)
%D：输入数据，每行一个样本，每列一个维度
%k：聚类类别数
%c：列向量，聚类类别标号
n=size(D,1);
c=zeros(n,1);
seed=randsample(n,k);   %选择初始样本点
center=D(seed,:);
while true
    c_last=c;
    for i=1:n
        dist=inf;
        for j=1:k
            %寻找最小距离
            if norm(D(i,:)-center(j,:))<dist
                dist=norm(D(i,:)-center(j,:));
                c(i)=j;
            end
        end
    end
    if all(c==c_last)   %是否和上一轮相同
        break
    end
    for i=1:k
        center(i,:)=mean(D(c==i,:),1);  %更新中心点
    end
end
end
%% 基础示例，面向过程
D=[0,3;
    0,2;
    1,2;
    0,0;
    1,0];
k=2;
c=kmeans(D,k);
disp(c);

%% 基础示例，面向对象
k_obj=kmeans_oop(D,k);
k_obj.init;
k_obj.train;
k_obj.c

%% iris测试
iris=readtable("iris.data","FileType","text");
iris_obj=kmeans_oop(iris{:,1:4},3);
iris_obj.init;
iris_obj.train;
iris_obj.c
%% iris绘图
figure;
bar(iris_obj.c,'LineStyle','none');
xlabel('number');
ylabel('label');
figure;
scatter3(iris_obj.D(:,1),iris_obj.D(:,2),iris_obj.D(:,3), ...
    25,iris_obj.c,'filled');
xlabel('sepal length[cm]');
ylabel('sepal width[cm]');
zlabel('petal length[cm]');
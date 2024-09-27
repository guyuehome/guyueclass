% 拟合散点
% 作者： Ally
% 日期： 2021/07/03
clc
clear
close all

%% 根据散点获得拟合曲线三维路径
x_seq = [0,1,2,3,4];
y_seq = [5,2,3,4,1];
z_seq = [3,4,5,2,0];

% 利用spline函数进行拟合插值
k = length(x_seq);
t_seq = linspace(0,1,k);
T_seq = linspace(0,1,100);
X_seq = spline(t_seq,x_seq,T_seq);
Y_seq = spline(t_seq,y_seq,T_seq);
Z_seq = spline(t_seq,z_seq,T_seq);

% 画拟合曲线图
figure
hold on
scatter3(x_seq, y_seq, z_seq, 100, 'bs','MarkerFaceColor','g')
plot3(X_seq, Y_seq, Z_seq, 'r','LineWidth',2)
grid on
title('散点拟合曲线')

%% 计算曲线的曲率、挠率
% 计算三阶导数
f = [X_seq; Y_seq; Z_seq];          % 表示函数
delta = 1 / length(X_seq);
f1 = gradient(f)./delta;            % 一阶导
f2 = gradient(f1)./delta;           % 二阶导
f3 = gradient(f2)./delta;           % 三阶导
f1 = f1';
f2 = f2';
f3 = f3';

% 曲率、挠率
v = cross(f1,f2,2);                % 一阶导与二阶导做外积
e = dot(f3,v,2);                   %（r',r'',r'''）混合积
c = zeros(length(T_seq),1);        % 定义矩阵c储存一阶导二阶导叉乘模长，d储存一阶导模长
d = c;
for i = 1:length(f)
    c(i) = norm(v(i,:));             % 一阶导二阶导外积的模长
    d(i) = norm(f1(i,:));            % 一阶导模长
end
k = c./(d.^3);                     % 曲率
torsion = e./c.^2;                 % 挠率

%% 画图
% 曲率图
figure
plot(k, 'r','LineWidth',2)
title('曲率图')

% 挠率图
figure
plot(torsion, 'r','LineWidth',2)
title('挠率图')
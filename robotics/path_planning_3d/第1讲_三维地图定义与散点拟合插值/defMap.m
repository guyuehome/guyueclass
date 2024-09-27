% 三维地图定义
% 作者： Ally
% 日期： 2021/07/03
clc
clear
close all

%% 初始化地形信息
mapRange = [100,100,100];           % 地图长、宽、高范围
N = 10;                             % 山峰个数
peaksInfo = struct;                 % 初始化山峰特征信息结构体
peaksInfo.center = [];              % 山峰中心
peaksInfo.range = [];               % 山峰区域
peaksInfo.height = [];              % 山峰高度
peaksInfo = repmat(peaksInfo,N,1);

%% 随机生成N个山峰的特征参数
for i = 1:N
    peaksInfo(i).center = [mapRange(1) * (rand*0.8+0.2), mapRange(2) * (rand*0.8+0.2)];
    peaksInfo(i).height = mapRange(3) * (rand*0.7+0.3);
    peaksInfo(i).range = mapRange*0.1*(rand*0.7+0.3);
end

%% 计算山峰曲面值
peaksData = [];
for x = 1:mapRange(1)
    for y = 1:mapRange(2)
        sum = 0;
        for k = 1:N
            h_i = peaksInfo(k).height;
            x_i = peaksInfo(k).center(1);
            y_i = peaksInfo(k).center(2);
            x_si = peaksInfo(k).range(1);
            y_si = peaksInfo(k).range(2);
            sum = sum + h_i * exp(-((x-x_i)/x_si)^2 - ((y-y_i)/y_si)^2);
        end
        peaksData(x,y) = sum;
    end
end

%% 构造曲面网格，用于后期MAP图插值判断三维路径是否与山峰交涉

% x列向量
x = [];
for i = 1:mapRange(1)
    x = [x; ones(mapRange(2),1) * i];
end

% y列向量
y = (1:mapRange(2))';
y = repmat(y,length(peaksData(:))/length(y),1);

% peaksData列向量
peaksData = reshape(peaksData,length(peaksData(:)),1);

% 构造X/Y/Z网格数据
[X,Y,Z] = griddata(x,y,peaksData,...
    linspace(min(x),max(x),100)',...
    linspace(min(y),max(y),100));

%% 画山峰曲面
surf(X,Y,Z)      % 画曲面图
shading flat     % 各小曲面之间不要网格

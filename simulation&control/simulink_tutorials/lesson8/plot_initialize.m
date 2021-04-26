function plot_initialize(xlab, ylab, tit)
%自定义绘图函数，方便绘图

% 创建 figure
figure1 = figure('PaperSize',[20 30]);

% 创建 axes
axes1 = axes('Parent',figure1,'FontSize',16,'FontName','Times New Roman');
box(axes1,'on');
hold(axes1,'all');

% 创建 xlabel
xlabel({xlab},'FontSize',16);

% 创建 ylabel
ylabel({ylab},'FontSize',16);

% 创建 title
title({tit},'FontSize',16);

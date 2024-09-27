function plotFigure(startPos,goalPos,X,Y,Z, GlobalBest)

% 画起点和终点
scatter3(startPos(1), startPos(2), startPos(3),100,'bs','MarkerFaceColor','y')
hold on
scatter3(goalPos(1), goalPos(2), goalPos(3),100,'kp','MarkerFaceColor','y')

% 画山峰曲面
surf(X,Y,Z)      % 画曲面图
shading flat     % 各小曲面之间不要网格

% 画路径
path = GlobalBest.path;
pos = GlobalBest.pos;
scatter3(pos(:,1), pos(:,2), pos(:,3), 'go');
plot3(path(:,1), path(:,2),path(:,3), 'r','LineWidth',2);

hold off
grid on


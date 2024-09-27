function [flag,fitness,path] = calFitness(startPos, goalPos,X,Y,Z, pos)
% 利用三次样条拟合散点
x_seq=[startPos(1), pos.x, goalPos(1)];
y_seq=[startPos(2), pos.y, goalPos(2)];
z_seq=[startPos(3), pos.z, goalPos(3)];

k = length(x_seq);
i_seq = linspace(0,1,k);
I_seq = linspace(0,1,100);
X_seq = spline(i_seq,x_seq,I_seq);
Y_seq = spline(i_seq,y_seq,I_seq);
Z_seq = spline(i_seq,z_seq,I_seq);
path = [X_seq', Y_seq', Z_seq'];

% 判断生成的曲线是否与与障碍物相交
flag = 0;
for i = 2:size(path,1)
    x = path(i,1);
    y = path(i,2);
    z_interp = interp2(X,Y,Z,x,y);
    if path(i,3) < z_interp
        flag = 1;
        break
    end
end


%% 计算三次样条得到的离散点的路径长度（适应度）
dx = diff(X_seq);
dy = diff(Y_seq);
dz = diff(Z_seq);
fitness = sum(sqrt(dx.^2 + dy.^2 + dz.^2));
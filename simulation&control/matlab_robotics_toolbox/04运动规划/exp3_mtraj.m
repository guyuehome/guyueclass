%% 多维插值
% 应用场景：6轴机械臂的各关节插值
p = mtraj(@tpoly, [0 0 0 0 0 0], [2 1 0 1.5 -1 0.5], 30); 
p1 = mtraj(@lspb, [0 0 0 0 0 0], [2 1 0 1.5 -1 0.5], 30); 
figure(1)
plot(p)
figure(2)
plot(p1)

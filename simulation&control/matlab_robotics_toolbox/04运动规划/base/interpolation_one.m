%% 五次多项式插值
p0 = -1;
p1 = 2;
[p,pd,pdd] = tpoly(p0, p1, 50);
figure(1)
subplot(3,1,1); plot(p); xlabel('Time'); ylabel('p-位置');
subplot(3,1,2); plot(pd); xlabel('Time'); ylabel('pd-速度');
subplot(3,1,3); plot(pdd); xlabel('Time'); ylabel('pdd-加速度');
%% 分段插值
[p,pd,pdd] = lspb(p0, p1, 50);
figure(2)
subplot(3,1,1); plot(p); xlabel('Time'); ylabel('p-位置');
subplot(3,1,2); plot(pd); xlabel('Time'); ylabel('pd-速度');
subplot(3,1,3); plot(pdd); xlabel('Time'); ylabel('pdd-加速度');
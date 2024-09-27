%% 一种公认的较理想的选择是采用混合曲线轨迹，
%即由中间的恒速段平直线加上两侧的加速段和减速段多项式曲线构成的轨迹。 
[p,pd,pdd] = lspb(p0, p1, 50);
 figure(2); subplot(3,1,1);
 plot(p);
 xlabel('Time');ylabel('p-位置'); 
subplot(3,1,2); 
plot(pd);
xlabel('Time'); ylabel('pd-速度'); 
subplot(3,1,3); 
plot(pdd); 
xlabel('Time');
 ylabel('pdd-加速度');

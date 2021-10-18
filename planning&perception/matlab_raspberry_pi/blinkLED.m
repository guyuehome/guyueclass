function blinkLED()%#codegen

% 和树莓派建立通信
r = raspi;

disp('start');

% 遍历十次
for i=1:10
    disp(i); % 输出i
    
    writeLED(r,'LED0',0); % 关闭LED0
    
    pause(0.5); % 等待0.5秒
    
    writeLED(r,'LED0',1); % 打开LED0
    
    pause(0.5); % 等待0.5秒
end    

disp('finish');
end
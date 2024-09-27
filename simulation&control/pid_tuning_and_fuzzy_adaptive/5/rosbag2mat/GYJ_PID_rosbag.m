%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 日期 2022/03/28
% 作者 DandD 董昊天
% 描述 解析ros包，辨识系统参数，1训练集 2验证集
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 清除 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear;
clc;
opengl software;
%% 导入ROS包 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bag1 = rosbag('2022-03-27-14-05-07.bag');
bag2 = rosbag('2022-03-27-14-07-23.bag');
%% 按话题提取数据 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ctrl1    = select(bag1,'Topic','/ctrl');
motorne1 = select(bag1,'Topic','/motorne');

ctrlStructs1     = readMessages(ctrl1,'DataFormat','struct');
motorneStructs1  = readMessages(motorne1,'DataFormat','struct');

ctrlt1    = ctrl1.MessageList.Time-ctrl1.MessageList.Time(1);
motornet1 = motorne1.MessageList.Time-motorne1.MessageList.Time(1);

ctrlData1    = cellfun(@(m) double(m.Data),ctrlStructs1);
motorneData1 = cellfun(@(m) double(m.Data),motorneStructs1);
%--------------------------------------------------------------------------%
ctrl2    = select(bag2,'Topic','/ctrl');
motorne2 = select(bag2,'Topic','/motorne');

ctrlStructs2     = readMessages(ctrl2,'DataFormat','struct');
motorneStructs2  = readMessages(motorne2,'DataFormat','struct');

ctrlt2    = ctrl2.MessageList.Time-ctrl2.MessageList.Time(1);
motornet2 = motorne2.MessageList.Time-motorne2.MessageList.Time(1);

ctrlData2    = cellfun(@(m) double(m.Data),ctrlStructs2);
motorneData2 = cellfun(@(m) double(m.Data),motorneStructs2);

for i = 1:length(motorneData1)
    if motorneData1(i)<0
        motorneData1(i) = motorneData1(i-1);
    end
end
motornet1 = motornet1(1:end-1);
motorneData1 = motorneData1(1:end-1);
for i = 1:length(motorneData2)
    if motorneData2(i)<0
        motorneData2(i) = motorneData2(i-1);
    end
end

%% 画图
figure(1);
subplot(2,1,1);
plot(ctrlt1,ctrlData1/2.56);
xlabel('t(s)');
ylabel('PWM(%)');
title('训练集');
subplot(2,1,2);
plot(motornet1,motorneData1*60);
xlabel('t(s)');
ylabel('ne(rpm)');

figure(2);
subplot(2,1,1);
plot(ctrlt2,ctrlData2/2.56);
xlabel('t(s)');
ylabel('PWM(%)');
title('验证集');
subplot(2,1,2);
plot(motornet2,motorneData2*60);
xlabel('t(s)');
ylabel('ne(rpm)');

figure(3);
plot(ctrlData1/2.56,motorneData1*60,'r*-');
hold on;
plot(ctrlData2/2.56,motorneData2*60,'bo-');
xlabel('PWM(%)');
ylabel('ne(rpm)');
legend('训练集','验证集');
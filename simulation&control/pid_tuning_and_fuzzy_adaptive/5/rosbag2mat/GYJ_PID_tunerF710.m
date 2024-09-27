%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 日期 2022/03/28
% 作者 DandD 董昊天
% 描述 解析ros包 F710 tuner
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 清除 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear;
clc;
opengl software;
%% 导入ROS包 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bag1 = rosbag('2022-03-27-18-08-49.bag');
%% 按话题提取数据 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ctrl1    = select(bag1,'Topic','/ctrl');
motorne1 = select(bag1,'Topic','/motorne');
params1  = select(bag1,'Topic','/params');

ctrlStructs1     = readMessages(ctrl1,'DataFormat','struct');
motorneStructs1  = readMessages(motorne1,'DataFormat','struct');
paramsStructs1   = readMessages(params1,'DataFormat','struct');

ctrlt1    = ctrl1.MessageList.Time-ctrl1.MessageList.Time(1);
motornet1 = motorne1.MessageList.Time-motorne1.MessageList.Time(1);

ctrlData1    = cellfun(@(m) double(m.Data),ctrlStructs1);
motorneData1 = cellfun(@(m) double(m.Data),motorneStructs1);
PData1 = cellfun(@(m) double(m.Position.X),paramsStructs1);
IData1 = cellfun(@(m) double(m.Position.Y),paramsStructs1);
DData1 = cellfun(@(m) double(m.Position.Z),paramsStructs1);
PData1(1)
IData1(1)
DData1(1)

for i = 1:length(motorneData1)
    if motorneData1(i)<0
        motorneData1(i) = motorneData1(i-1);
    end
end

realctrl = [ctrlt1,ctrlData1];
realdata = [motornet1,motorneData1];
motornet1 = motornet1(1:end-2);
motorneData1 = motorneData1(1:end-2);
%% 画图
figure(1);
subplot(2,1,1);
plot(ctrlt1,ctrlData1/2.56);
xlabel('t(s)');
ylabel('PWM(%)');
subplot(2,1,2);
plot(motornet1,motorneData1*60);
xlabel('t(s)');
ylabel('ne(rpm)');

figure(2);
plot(ctrlData1/2.56,motorneData1*60,'r*-');
xlabel('PWM(%)');
ylabel('ne(rpm)');
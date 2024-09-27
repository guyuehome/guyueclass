function TL = updateTL(TL_ini,t,ax)
% 四相位信号交叉口，周期长度为120s
TL = TL_ini;
t = mod(t,120);
t = round(t,1);
%% 相位1：东西直行，绿30s，黄3s
if t >= 0 && t <30
    TL(1).stright.state = 'green';
    TL(3).stright.state = 'green';
elseif t >= 30 && t < 33
    TL(1).stright.state = 'yellow';
    TL(3).stright.state = 'yellow';
end

%% 相位2：东西左转，绿25s，黄2s
if t >= 33 && t < 58
    TL(1).left.state = 'green';
    TL(3).left.state = 'green';
elseif t >= 58 && t < 60
    TL(1).left.state = 'yellow';
    TL(3).left.state = 'yellow';
end        

%% 相位3：南北直行，绿30s，黄3s
if t >= 60 && t < 90
    TL(2).stright.state = 'green';
    TL(4).stright.state = 'green';
elseif t >= 90 && t < 93
    TL(2).stright.state = 'yellow';
    TL(4).stright.state = 'yellow';
end    

%% 相位4：南北左转，绿25s，黄3s
if t >= 90 && t < 118
    TL(2).left.state = 'green';
    TL(4).left.state = 'green';
elseif t >= 118 && t < 120
    TL(2).left.state = 'yellow';
    TL(4).left.state = 'yellow';
end    

%% 更新信号灯颜色
if ismember(t,[0.1 30 33 58 60 90 93 118 120])
    plotTL(TL,ax)
end
function refPath = getRefPath(startPos, goalPos,entryIdx,laneIdx)
% 构造二次贝塞尔曲线的三个控制点
P0 = startPos(1:2);
P2 = goalPos(1:2);
switch laneIdx
    case 1
        P1 = [0, 0];
    case 2
        P1 = (P0 + P2) / 2;
    case 3
        if entryIdx == 1 || entryIdx == 5
            P1 = [startPos(1),goalPos(2)];
        else
            P1 = [goalPos(1),startPos(2)];
        end
end

% 利用二次贝塞尔曲线模拟转弯路径
refPath = [];
for t = 0:0.01:1
    P_t_1 = (1-t)*P0 + t*P1;
    P_t_2 = (1-t)*P1 + t*P2;
    P_t_3 = (1-t)*P_t_1 + t*P_t_2;
    refPath(end+1,:) = P_t_3;
end

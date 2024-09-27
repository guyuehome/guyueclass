function [myPos_new,myVel_new,myYaw_new] = updateByPP(refPath,myPos,myVel)
%% 相关参数定义
dt = evalin('base','dt');
Ld0 = 1;               % Ld0是预瞄距离的下限值

% 车辆初始状态定义
myPos = myPos(1:2);
v = norm(myVel(1:2));

%% 寻找预瞄距离范围内最近路径点
% 找到距离当前位置最近的一个参考轨迹点的序号
dist = sqrt((refPath(:,1)-myPos(1)).^2 + (refPath(:,2)-myPos(2)).^2 );
[~,idx] = min(dist); 

% 从该点开始向轨迹前方搜索，找到与预瞄距离最相近的一个轨迹点
L_steps = 0;           % 参考轨迹上几个相邻点的累计距离
Ld = v*dt + Ld0;       
sizeOfRefPos = size(refPath,1);
while L_steps < Ld && idx < sizeOfRefPos
    L_steps = L_steps + norm(refPath(idx + 1,:) - refPath(idx,:));
    idx = idx+1;
end
lookaheadPoint = refPath(idx,:);
deltaPos = lookaheadPoint - myPos(1:2);
yaw = atan2(deltaPos(2), deltaPos(1));

%% 更新状态量
myPos_new = zeros(1,3);
myVel_new = zeros(1,3);
myPos_new(1:2) = lookaheadPoint;
myYaw_new = yaw;
myYaw_new = myYaw_new * 180/ pi;

% 调整航向角
if myYaw_new > 180
    myYaw_new = myYaw_new - 360;
end
if myYaw_new < -180
    myYaw_new = myYaw_new + 360;
end
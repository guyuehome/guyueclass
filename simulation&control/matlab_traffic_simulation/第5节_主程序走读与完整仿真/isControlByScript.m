function [flag_control,a,flag_stop] = isControlByScript(myPos_LLV,myVel_LLV,myInfo,...
    Pos_link_LLV,V_link_LLV)
% 当临近路口或位于路口内时，不再使用ststeflow进行控制，启用本函数
%% 参数
% 从工作空间导入参数
TL = evalin('base','TL');
laneWidth = evalin('base','laneWidth');
numLanes = evalin('base','numLanes');
par = evalin('base','par');
solidLineLength = evalin('base','solidLineLength');

% 其他参数
flag_control = 0;                                  % 是否对车辆进行单独控制的标识符
flag_stop = 0;                                     % 停车信号
a = 0;                                             % 加速度
L0 = 7;                                            % 停车时的前后最小车距
v_turn = 8;                                        % 车辆经过路口的目标速度
boundLine = 0 : laneWidth : numLanes*laneWidth;    % 车道分界线横坐标
distToTL = par.laneLength - ...
    par.laneWidth*par.numLanes - 2 - myPos_LLV(1); % 车辆到红绿灯的距离
myLaneIdx = myInfo.laneIdx;                        % 车道编号
if mod(myInfo.linkIdx,2) == 1                      % 红绿灯编号
    idx_TL = (myInfo.linkIdx+1)/2;
else
    idx_TL = myInfo.linkIdx/2;
end

%% 判断
if distToTL <= solidLineLength && myLaneIdx ~= 3 && mod(myInfo.linkIdx,2) == 1
    flag_control = 1;    % 将控制标识符设为1
    
    % 将位置坐标按照车道分类
    Pos_lane_LLV_1 = [];
    Pos_lane_LLV_2 = [];
    V_lane_LLV_1 = [];
    V_lane_LLV_2 = [];
    for i = 1:size(Pos_link_LLV,1)
        temp = [boundLine,Pos_link_LLV(i,2)];
        temp = sort(temp);
        [~,idx] = ismember(Pos_link_LLV(i,2),temp);
        switch idx
            case 2
                Pos_lane_LLV_1(end+1,:) = Pos_link_LLV(i,:);
                V_lane_LLV_1(end+1,:) = V_link_LLV(i,:);
            case 3
                Pos_lane_LLV_2(end+1,:) = Pos_link_LLV(i,:);
                V_lane_LLV_2(end+1,:) = V_link_LLV(i,:);
        end
    end
    
    % 判断本车是否是头车
    if myLaneIdx == 1 && myPos_LLV(1) == max(Pos_lane_LLV_1(:,1)) ...
            || myLaneIdx == 2 && myPos_LLV(1) == max(Pos_lane_LLV_2(:,1))
        % 如果是头车,且本车道为红灯状态，计算减速度
        if myInfo.laneIdx == 1 && TL(idx_TL).left.state == "red" ...
                || myInfo.laneIdx == 2 && TL(idx_TL).stright.state == "red"
            if distToTL < 1
                flag_stop = 1;
                a = 0;
            else
                a = -myVel_LLV(1)^2 / (2*distToTL);
            end
        end
        
        % 如果是头车,且本车道为绿灯状态，计算加/减速度
        if myInfo.laneIdx == 1 && TL(idx_TL).left.state == "green" ...
                || myInfo.laneIdx == 2 && TL(idx_TL).stright.state == "green"
            a = (v_turn^2- myVel_LLV(1)^2) / (2*distToTL);
        end
        
    else
        % 若不是头车，先计算前方车辆的位置、速度再计算加/减速度
        Pos_lane_LLV = eval("Pos_lane_LLV_" + num2str(myLaneIdx));
        V_lane_LLV = eval("V_lane_LLV_" + num2str(myLaneIdx));
        [Pos_lane_LLV_sort,sortIdx] = sort(Pos_lane_LLV(:,1), 'descend');
        [~,myIdx] = ismember(myPos_LLV(1),Pos_lane_LLV_sort);
        frontCarPos = Pos_lane_LLV(sortIdx(myIdx-1),:);
        frontCarV = V_lane_LLV(sortIdx(myIdx-1),:);
        
        % 再计算本车和前方车辆的距离，若大于最小安全车距，则产生加/减速度
        L = frontCarPos(1) - myPos_LLV(1);
        if L > L0
            a = -0.5 * (myVel_LLV(1) - frontCarV(1))^2 / (L - L0);
        else
            % 对异常情况打补丁：越过L0后若速度大于前车速度，可能会与前车相撞
            a = -0.5 * (myVel_LLV(1) - frontCarV(1))^2 / L;
        end
    end
end



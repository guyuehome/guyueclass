function [pos_transfer,v_transfer] = transferCoord(pos,v,flag,linkIdx)
% 参数
laneLength = evalin('base','laneLength');
numLanes = evalin('base','numLanes');
laneWidth = evalin('base','laneWidth');
pos_transfer = zeros(size(pos,1),3);
v_transfer = zeros(size(pos,1),3);

% 选择转换规则
switch flag
    case 1
        % 若flag=1，则是XYZ坐标向LLV坐标转换
        switch linkIdx
            % 选择当前位置的路段编号
            case 1
                pos_transfer(:,1) = laneLength - abs(pos(:,2));
                pos_transfer(:,2) = abs(pos(:,1));
                v_transfer(:,1) = v(:,2);
                v_transfer(:,2) = -v(:,1);
            case 2
                pos_transfer(:,1) = abs(pos(:,2)) - laneWidth*numLanes;
                pos_transfer(:,2) = abs(pos(:,1));
                v_transfer(:,1) = -v(:,2);
                v_transfer(:,2) = v(:,1);
            case 3
                pos_transfer(:,1) = laneLength - pos(:,1);
                pos_transfer(:,2) = pos(:,2);
                v_transfer = -v;
            case 4
                pos_transfer(:,1) = pos(:,1) - laneWidth*numLanes;
                pos_transfer(:,2) = abs(pos(:,2));
                v_transfer = v;
                
            case 5
                pos_transfer(:,1) = laneLength - pos(:,2);
                pos_transfer(:,2) = abs(pos(:,1));
                v_transfer(:,1) = -v(:,2);
                v_transfer(:,2) = v(:,1);
            case 6
                pos_transfer(:,1) = pos(:,2) - laneWidth*numLanes;
                pos_transfer(:,2) = pos(:,1);
                v_transfer(:,1) = v(:,2);
                v_transfer(:,2) = -v(:,1);
                
            case 7
                pos_transfer(:,1) = laneLength - abs(pos(:,1));
                pos_transfer(:,2) = abs(pos(:,2));
                v_transfer = v;
            case 8
                pos_transfer(:,1) = abs(pos(:,1) + laneWidth*numLanes);
                pos_transfer(:,2) = pos(:,2);
                v_transfer = -v;
        end
        
        
    case 2
        % 若flag=2，则LLV坐标向XYZ坐标转换
        switch linkIdx
            % 选择当前位置的路段编号
            case 1
                pos_transfer(:,1) = pos(:,2);
                pos_transfer(:,2) = pos(:,1)-laneLength;
                v_transfer(:,1) = -v(:,2);
                v_transfer(:,2) = v(:,1);
            case 2
                pos_transfer(:,1) = -pos(:,2);
                pos_transfer(:,2) = -(pos(:,1)+laneWidth*numLanes);
                v_transfer(:,1) = v(:,2);
                v_transfer(:,2) = -v(:,1);
            case 3
                pos_transfer(:,1) = laneLength - pos(:,1);
                pos_transfer(:,2) = pos(:,2);
                v_transfer = -v;
            case 4
                pos_transfer(:,1) = pos(:,1) + laneWidth*numLanes;
                pos_transfer(:,2) = -pos(:,2);
                v_transfer = v;
            case 5
                pos_transfer(:,1) = -pos(:,2);
                pos_transfer(:,2) = laneLength - abs(pos(:,1));
                v_transfer(:,1) = v(:,2);
                v_transfer(:,2) = -v(:,1);
            case 6
                pos_transfer(:,1) = pos(:,2);
                pos_transfer(:,2) = pos(:,1) + laneWidth*numLanes;
                v_transfer(:,1) = -v(:,2);
                v_transfer(:,2) = v(:,1);
            case 7
                pos_transfer(:,1) = pos(:,1) - laneLength;
                pos_transfer(:,2) = -pos(:,2);
                v_transfer = v;
            case 8
                pos_transfer(:,1) = -(pos(:,1)  + laneWidth*numLanes);
                pos_transfer(:,2) = pos(:,2);
                v_transfer = -v;
        end
end

function [vehInfo,Pos_all_XYZ,V_all_XYZ,posInLink,vInLink] = getVehsInfo...
    (Scenario,vehInfo,par,exitID)
%% 从场景文件Scenario获取当前所有车辆的位置和速度信息
% 位姿信息
rawPoses = actorPoses(Scenario);             % 获得场景中每辆车的位姿信息
rawPoses = struct2cell(rawPoses);            % 将元胞数组重置为N×3矩阵

% 位置
Pos_all_XYZ = rawPoses(2,:);                     % 后缀为XYZ表达场景地图中的坐标，后缀为LLV表达为纵向、横向、垂向坐标
Pos_all_XYZ = reshape([Pos_all_XYZ{:}], 3,[])';

% 速度
V_all_XYZ = rawPoses(3,:);
V_all_XYZ = reshape([V_all_XYZ{:}], 3,[])';

% 删掉已经达到路段末端，“消失”在场景中的车辆
Pos_all_XYZ(exitID,:) = [];
V_all_XYZ(exitID,:) = [];

%% 统计每条路段当前的车辆
% 先定义8块道路区域，编号1~8依次对应右进、右出、上进、上出、左进、左出、下进、下出
laneZone{1,1} = [0, -par.laneLength; par.laneWidth*par.numLanes, -par.laneLength;
    par.laneWidth*par.numLanes, -par.laneWidth*par.numLanes; 0,  -par.laneWidth*par.numLanes];
laneZone{1,2} = [0, -par.laneLength; 0,  -par.laneWidth*par.numLanes;
    -par.laneWidth*par.numLanes, -par.laneWidth*par.numLanes; -par.laneWidth*par.numLanes, -par.laneLength];
laneZone{1,3} = [par.laneLength, par.laneWidth*par.numLanes; par.laneWidth*par.numLanes, par.laneWidth*par.numLanes;
    par.laneWidth*par.numLanes, 0; par.laneLength, 0];
laneZone{1,4} = [par.laneLength, 0; par.laneWidth*par.numLanes, 0;
    par.laneWidth*par.numLanes, -par.laneWidth*par.numLanes; par.laneLength, -par.laneWidth*par.numLanes];
laneZone{1,5} = [0 par.laneLength; -par.laneWidth*par.numLanes, par.laneLength;
    -par.laneWidth*par.numLanes, par.laneWidth*par.numLanes; 0, par.laneWidth*par.numLanes];
laneZone{1,6} = [0 par.laneLength; 0, par.laneWidth*par.numLanes;
    par.laneWidth*par.numLanes, par.laneWidth*par.numLanes; par.laneWidth*par.numLanes, par.laneLength];
laneZone{1,7} = [-par.laneLength 0; -par.laneLength, -par.laneWidth*par.numLanes;
    -par.laneWidth*par.numLanes, -par.laneWidth*par.numLanes; -par.laneWidth*par.numLanes, 0];
laneZone{1,8} = [-par.laneLength 0;  -par.laneWidth*par.numLanes, 0;
    -par.laneWidth*par.numLanes, par.laneWidth*par.numLanes;-par.laneLength par.laneWidth*par.numLanes];

% 车辆位于路段link的编号
posInLink = cell(1,8);
vInLink = cell(1,8);
for i = 1:size(vehInfo,1)
    flag = 0;
    for j = 1:8
        in = inpolygon(Pos_all_XYZ(i,1),Pos_all_XYZ(i,2), laneZone{j}(:,1),laneZone{j}(:,2));
        if in
            vehInfo(i).linkIdx = j;
            posInLink{1,j}(end+1,1:3) = Pos_all_XYZ(i,:);
            vInLink{1,j}(end+1,1:3) = V_all_XYZ(i,:);
            flag = 1;
            break
        end
    end
    
    % 若flag == 0，表明当前车辆正位于交叉路口内
    if flag == 0
        vehInfo(i).linkIdx = 0;
    end
end
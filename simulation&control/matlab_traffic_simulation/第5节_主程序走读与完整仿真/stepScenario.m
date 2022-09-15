function [Scenario,vehInfo,exitID] = stepScenario(par,Scenario,vehInfo,intersectionGoalPos,exitID)
%% 获取车辆信息
dt = Scenario.SampleTime;
[vehInfo,Pos_all_XYZ,V_all_XYZ,posInLink,vInLink] = getVehsInfo...
    (Scenario,vehInfo,par,exitID);

%% 步进场景中的每辆车
deleteIdx = []; % 考虑到车辆到达终点后，需要移除车辆，故需要在vehInfo删除的车辆编号
for i = 1:length(vehInfo)
    % 获取当前车辆的信息
    ID = vehInfo(i).ID;
    driver = vehInfo(i).Drivers;
    myPos_XYZ = Pos_all_XYZ(i,:);
    myVel_XYZ = V_all_XYZ(i,:);
    
    % 首先判断场景仿真时间是否大于车辆的出场时间，且小于退出时间，则执行车辆位置更新
    if Scenario.SimulationTime >= Scenario.Actors(ID).EntryTime...
            && Scenario.SimulationTime < Scenario.Actors(ID).ExitTime
        
        % 判断路段编号是否为0，若不为0表明正处于路段中，否则表明正处于交叉口内
        if vehInfo(i).linkIdx ~= 0
            % 获取当前车辆所在路段的所有车辆的位置速度信息，并将XYZ转为LLV坐标
            Pos_link_XYZ = posInLink{vehInfo(i).linkIdx};
            V_link_XYZ = vInLink{vehInfo(i).linkIdx};
            [Pos_link_LLV,V_link_LLV] = transferCoord(Pos_link_XYZ,...
                V_link_XYZ,1,vehInfo(i).linkIdx);
            
            % 计算本车在本路段link所有车辆位置的索引号
            [~,idx_me] = ismember(myPos_XYZ, Pos_link_XYZ,'rows');
            
            % 将本车的XYZ坐标转为LLV坐标
            [myPos_LLV,myVel_LLV] = transferCoord(myPos_XYZ,...
                myVel_XYZ,1,vehInfo(i).linkIdx);
            
            % 根据位置计算本车的车道lane编号
            temp = abs(par.laneCenters - myPos_LLV(2));
            [~, vehInfo(i).laneIdx] = min(temp);
            
            % 判断本车是否是本路段及本车道的头车，用于红灯减速
            [flag_control,a,flag_stop] = isControlByScript(myPos_LLV,myVel_LLV,...
                vehInfo(i),Pos_link_LLV,V_link_XYZ);
            
            % 判断flag_control的值,选择用程序脚本控制还是stateflow控制
            if flag_control == 1
                % 若为头车且本车道为红灯，则用脚本控制行驶
                if flag_stop == 1
                    % 若停车标识符为1，则让速度强制为0
                    myVel_LLV = [0,0,0];
                else
                    % 否则，根据加速度（正或负）计算速度
                    myVel_LLV(1) = max(myVel_LLV(1) + a * dt,0);
                    myVel_LLV(2) = 0;
                end
                % 根据加速度和速度更新位置
                myPos_LLV(1) = myPos_LLV(1) + myVel_LLV(1)*dt + 0.5*a*dt^2;

            else
                % 否则调用stateFlow状态机模型，并将结果重新刷新在driver里面：
                % 首先需要排除已经处于控制区的车辆
                [Pos_link_LLV_sf,V_link_LLV_sf] = ...
                    controlledBySfAreaVeh(vehInfo(i).linkIdx,Pos_link_LLV,V_link_LLV);
                [~,idx_me_sf] = ismember(myPos_LLV, Pos_link_LLV_sf,'rows');
                
                % 再调用stateFlow状态机模型
                step(driver,'positions',Pos_link_LLV_sf,...
                    'velocities',V_link_LLV_sf,...
                    'myVel',max(20,myVel_LLV(1)),...
                    'me',idx_me_sf);
                vehInfo(i).Drivers = driver;
                
                % 更新速度和位置
                myVel_LLV = driver.myVel;
                myVel_LLV(1) = max(0.2,myVel_LLV(1));
                myPos_LLV = myPos_LLV + myVel_LLV * Scenario.SampleTime;
            end
                        
            % 将更新之后的本车位置的LLV坐标转为XYZ坐标
            [myPos_XYZ,myVel_XYZ] = transferCoord(myPos_LLV,myVel_LLV,...
                2,vehInfo(i).linkIdx);
            
            % 重新赋值给场景，用于动态出图
            if ~isequal(myPos_XYZ,Scenario.Actors(ID).Position)
                deltaPos = myPos_XYZ - Scenario.Actors(ID).Position;
                myYaw = atan2(deltaPos(2), deltaPos(1));
                Scenario.Actors(ID).Yaw = myYaw * 180 / pi;
            end
            Scenario.Actors(ID).Velocity = myVel_XYZ;
            Scenario.Actors(ID).Position = myPos_XYZ;
            
            % 将本车刚更新的信息也要反馈到vehInfo中，用于下一个车辆的更新
            posInLink{vehInfo(i).linkIdx}(idx_me,:) = myPos_XYZ;
            vInLink{vehInfo(i).linkIdx}(idx_me,:) = myVel_XYZ;
            
            % 判断本车是否到达路段终点，从而更改从场景中消失的时间
            if mod(vehInfo(i).linkIdx,2) == 0 && myPos_LLV(1) > par.laneLength-par.laneWidth*par.numLanes
                Scenario.Actors(ID).ExitTime = Scenario.SimulationTime + dt;
                exitID(end+1) = ID;
                deleteIdx(end+1) = i;
            end
        else
            % 若当前车辆的路段编号为0，表明处于十字交叉口内
            % 为每一个车道的车辆重新规划一条转弯路径
            if vehInfo(i).passFlag == 0
                switch vehInfo(i).laneIdx
                    case 1
                        goalPos_XYZ =  intersectionGoalPos((vehInfo(i).entryIdx+1)/2).turnLeft;
                    case 2
                        goalPos_XYZ =  intersectionGoalPos((vehInfo(i).entryIdx+1)/2).turnStright;
                    case 3
                        goalPos_XYZ =  intersectionGoalPos((vehInfo(i).entryIdx+1)/2).turnRight;
                end
                vehInfo(i).turnPath = getRefPath(myPos_XYZ, goalPos_XYZ,...
                    vehInfo(i).entryIdx, vehInfo(i).laneIdx);
                vehInfo(i).passFlag = 1;
            else
                % 更新位置和航向角
                [myPos_new,myVel_new,myYaw_new] = updateByPP...
                    (vehInfo(i).turnPath,myPos_XYZ,myVel_XYZ);
                Scenario.Actors(ID).Position = myPos_new;
                Scenario.Actors(ID).Yaw = myYaw_new;
                Scenario.Actors(ID).Velocity = myVel_new;
            end
        end
    end
end
vehInfo(deleteIdx) = [];

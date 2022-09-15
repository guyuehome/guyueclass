function [Pos_link_LLV_sf,V_link_LLV_sf] = controlledBySfAreaVeh(linkIdx,Pos_link_LLV,V_link_LLV)
% 获取直接被StateFlow控制的车辆信息
par = evalin('base','par');
Pos_link_LLV_sf = Pos_link_LLV;
V_link_LLV_sf = V_link_LLV;
idx_uncontrolled = [];
for i = 1:size(Pos_link_LLV_sf,1)
    myPos_LLV = Pos_link_LLV_sf(i,:);
    distToTL = par.laneLength - par.laneWidth*par.numLanes - 2 - myPos_LLV(1); 
    if distToTL <= 30 && myPos_LLV(2) <= par.laneWidth*(par.numLanes-1) ...
            && mod(linkIdx,2) == 1
        idx_uncontrolled(end+1) = i;
    end
end
Pos_link_LLV_sf(idx_uncontrolled,:) = [];
V_link_LLV_sf(idx_uncontrolled,:) = [];
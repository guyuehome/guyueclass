function [Nodes,U] = UpdateEdgeCost(influencedNnodes,Nodes,U,km,rows,cols,s_start )

for i = 1:length(influencedNnodes)
    node = influencedNnodes(i);
    Nodes(node).rhs = inf;
    
    % 由于Nodes(node).rhs ~= Nodes(node).h，加入到queue中
    U(end+1,1) = node;
    U(end,2:3) = calculateKey(Nodes(node),s_start,km, rows, cols);
end
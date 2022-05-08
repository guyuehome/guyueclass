%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPEA120
% Project Title: Non-dominated Sorting Genetic Algorithm II (NSGA-II)
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function [pop, F]=SortPopulation(pop)

    % Sort Based on Crowding Distance
    [~, CDSO]=sort([pop.CrowdingDistance],'descend');
    pop=pop(CDSO);
    
    % Sort Based on Rank
    [~, RSO]=sort([pop.Rank]);
    pop=pop(RSO);
    
    % Update Fronts
    Ranks=[pop.Rank];
    MaxRank=max(Ranks);
    F=cell(MaxRank,1);
    for r=1:MaxRank
        F{r}=find(Ranks==r);
    end

end
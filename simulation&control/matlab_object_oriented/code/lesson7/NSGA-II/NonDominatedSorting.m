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

function [pop, F]=NonDominatedSorting(pop)

    nPop=numel(pop);

    for i=1:nPop
        pop(i).DominationSet=[];
        pop(i).DominatedCount=0;
    end
    
    F{1}=[];
    
    for i=1:nPop
        for j=i+1:nPop
            p=pop(i);
            q=pop(j);
            
            if Dominates(p,q)
                p.DominationSet=[p.DominationSet j];
                q.DominatedCount=q.DominatedCount+1;
            end
            
            if Dominates(q.Cost,p.Cost)
                q.DominationSet=[q.DominationSet i];
                p.DominatedCount=p.DominatedCount+1;
            end
            
            pop(i)=p;
            pop(j)=q;
        end
        
        if pop(i).DominatedCount==0
            F{1}=[F{1} i];
            pop(i).Rank=1;
        end
    end
    
    k=1;
    
    while true
        
        Q=[];
        
        for i=F{k}
            p=pop(i);
            
            for j=p.DominationSet
                q=pop(j);
                
                q.DominatedCount=q.DominatedCount-1;
                
                if q.DominatedCount==0
                    Q=[Q j]; %#ok
                    q.Rank=k+1;
                end
                
                pop(j)=q;
            end
        end
        
        if isempty(Q)
            break;
        end
        
        F{k+1}=Q; %#ok
        
        k=k+1;
        
    end
    

end